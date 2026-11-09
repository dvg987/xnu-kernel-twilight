/*
 * The VP9 decoder's backward probability adaptation (hakios.vp9.c), which this core leaves to the CPU: after a frame,
 * the firmware asks for it (VP9_ADAPT_PROB_REG 0xfd) with the frame's symbol counts and the context it started from in
 * memory. Amlogic's adapt_coef_probs (vvp9.c), the libvpx algorithm on the firmware's layout of the tables.
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/hakios.vp9.h>

static inline int
div_r32(int64_t m, int n)
{
	return (int)(m / n);
}

/*====================================================
 *========================================================================
 *vp9_prob define
 *========================================================================
 */
#define VP9_PARTITION_START      0
#define VP9_PARTITION_SIZE_STEP  (3 * 4)
#define VP9_PARTITION_ONE_SIZE   (4 * VP9_PARTITION_SIZE_STEP)
#define VP9_PARTITION_KEY_START  0
#define VP9_PARTITION_P_START    VP9_PARTITION_ONE_SIZE
#define VP9_PARTITION_SIZE       (2 * VP9_PARTITION_ONE_SIZE)
#define VP9_SKIP_START           (VP9_PARTITION_START + VP9_PARTITION_SIZE)
#define VP9_SKIP_SIZE            4 /* only use 3*/
#define VP9_TX_MODE_START        (VP9_SKIP_START+VP9_SKIP_SIZE)
#define VP9_TX_MODE_8_0_OFFSET   0
#define VP9_TX_MODE_8_1_OFFSET   1
#define VP9_TX_MODE_16_0_OFFSET  2
#define VP9_TX_MODE_16_1_OFFSET  4
#define VP9_TX_MODE_32_0_OFFSET  6
#define VP9_TX_MODE_32_1_OFFSET  9
#define VP9_TX_MODE_SIZE         12
#define VP9_COEF_START           (VP9_TX_MODE_START+VP9_TX_MODE_SIZE)
#define VP9_COEF_BAND_0_OFFSET   0
#define VP9_COEF_BAND_1_OFFSET   (VP9_COEF_BAND_0_OFFSET + 3 * 3 + 1)
#define VP9_COEF_BAND_2_OFFSET   (VP9_COEF_BAND_1_OFFSET + 6 * 3)
#define VP9_COEF_BAND_3_OFFSET   (VP9_COEF_BAND_2_OFFSET + 6 * 3)
#define VP9_COEF_BAND_4_OFFSET   (VP9_COEF_BAND_3_OFFSET + 6 * 3)
#define VP9_COEF_BAND_5_OFFSET   (VP9_COEF_BAND_4_OFFSET + 6 * 3)
#define VP9_COEF_SIZE_ONE_SET    100 /* ((3 +5*6)*3 + 1 padding)*/
#define VP9_COEF_4X4_START       (VP9_COEF_START + 0 * VP9_COEF_SIZE_ONE_SET)
#define VP9_COEF_8X8_START       (VP9_COEF_START + 4 * VP9_COEF_SIZE_ONE_SET)
#define VP9_COEF_16X16_START     (VP9_COEF_START + 8 * VP9_COEF_SIZE_ONE_SET)
#define VP9_COEF_32X32_START     (VP9_COEF_START + 12 * VP9_COEF_SIZE_ONE_SET)
#define VP9_COEF_SIZE_PLANE      (2 * VP9_COEF_SIZE_ONE_SET)
#define VP9_COEF_SIZE            (4 * 2 * 2 * VP9_COEF_SIZE_ONE_SET)
#define VP9_INTER_MODE_START     (VP9_COEF_START+VP9_COEF_SIZE)
#define VP9_INTER_MODE_SIZE      24 /* only use 21 ( #*7)*/
#define VP9_INTERP_START         (VP9_INTER_MODE_START+VP9_INTER_MODE_SIZE)
#define VP9_INTERP_SIZE          8
#define VP9_INTRA_INTER_START    (VP9_INTERP_START+VP9_INTERP_SIZE)
#define VP9_INTRA_INTER_SIZE     4
#define VP9_INTERP_INTRA_INTER_START  VP9_INTERP_START
#define VP9_INTERP_INTRA_INTER_SIZE   (VP9_INTERP_SIZE + VP9_INTRA_INTER_SIZE)
#define VP9_COMP_INTER_START     \
		(VP9_INTERP_INTRA_INTER_START+VP9_INTERP_INTRA_INTER_SIZE)
#define VP9_COMP_INTER_SIZE      5
#define VP9_COMP_REF_START       (VP9_COMP_INTER_START+VP9_COMP_INTER_SIZE)
#define VP9_COMP_REF_SIZE        5
#define VP9_SINGLE_REF_START     (VP9_COMP_REF_START+VP9_COMP_REF_SIZE)
#define VP9_SINGLE_REF_SIZE      10
#define VP9_REF_MODE_START       VP9_COMP_INTER_START
#define VP9_REF_MODE_SIZE        \
		(VP9_COMP_INTER_SIZE+VP9_COMP_REF_SIZE+VP9_SINGLE_REF_SIZE)
#define VP9_IF_Y_MODE_START      (VP9_REF_MODE_START+VP9_REF_MODE_SIZE)
#define VP9_IF_Y_MODE_SIZE       36
#define VP9_IF_UV_MODE_START     (VP9_IF_Y_MODE_START+VP9_IF_Y_MODE_SIZE)
#define VP9_IF_UV_MODE_SIZE      92 /* only use 90*/
#define VP9_MV_JOINTS_START      (VP9_IF_UV_MODE_START+VP9_IF_UV_MODE_SIZE)
#define VP9_MV_JOINTS_SIZE       3
#define VP9_MV_SIGN_0_START      (VP9_MV_JOINTS_START+VP9_MV_JOINTS_SIZE)
#define VP9_MV_SIGN_0_SIZE       1
#define VP9_MV_CLASSES_0_START   (VP9_MV_SIGN_0_START+VP9_MV_SIGN_0_SIZE)
#define VP9_MV_CLASSES_0_SIZE    10
#define VP9_MV_CLASS0_0_START    (VP9_MV_CLASSES_0_START+VP9_MV_CLASSES_0_SIZE)
#define VP9_MV_CLASS0_0_SIZE     1
#define VP9_MV_BITS_0_START      (VP9_MV_CLASS0_0_START+VP9_MV_CLASS0_0_SIZE)
#define VP9_MV_BITS_0_SIZE       10
#define VP9_MV_SIGN_1_START      (VP9_MV_BITS_0_START+VP9_MV_BITS_0_SIZE)
#define VP9_MV_SIGN_1_SIZE       1
#define VP9_MV_CLASSES_1_START   \
			(VP9_MV_SIGN_1_START+VP9_MV_SIGN_1_SIZE)
#define VP9_MV_CLASSES_1_SIZE    10
#define VP9_MV_CLASS0_1_START    \
			(VP9_MV_CLASSES_1_START+VP9_MV_CLASSES_1_SIZE)
#define VP9_MV_CLASS0_1_SIZE     1
#define VP9_MV_BITS_1_START      \
			(VP9_MV_CLASS0_1_START+VP9_MV_CLASS0_1_SIZE)
#define VP9_MV_BITS_1_SIZE       10
#define VP9_MV_CLASS0_FP_0_START \
			(VP9_MV_BITS_1_START+VP9_MV_BITS_1_SIZE)
#define VP9_MV_CLASS0_FP_0_SIZE  9
#define VP9_MV_CLASS0_FP_1_START \
			(VP9_MV_CLASS0_FP_0_START+VP9_MV_CLASS0_FP_0_SIZE)
#define VP9_MV_CLASS0_FP_1_SIZE  9
#define VP9_MV_CLASS0_HP_0_START \
			(VP9_MV_CLASS0_FP_1_START+VP9_MV_CLASS0_FP_1_SIZE)
#define VP9_MV_CLASS0_HP_0_SIZE  2
#define VP9_MV_CLASS0_HP_1_START \
			(VP9_MV_CLASS0_HP_0_START+VP9_MV_CLASS0_HP_0_SIZE)
#define VP9_MV_CLASS0_HP_1_SIZE  2
#define VP9_MV_START             VP9_MV_JOINTS_START
#define VP9_MV_SIZE              72 /*only use 69*/

#define VP9_TOTAL_SIZE           (VP9_MV_START + VP9_MV_SIZE)


/*========================================================================
 *	vp9_count_mem define
 *========================================================================
 */
#define VP9_COEF_COUNT_START           0
#define VP9_COEF_COUNT_BAND_0_OFFSET   0
#define VP9_COEF_COUNT_BAND_1_OFFSET   \
			(VP9_COEF_COUNT_BAND_0_OFFSET + 3*5)
#define VP9_COEF_COUNT_BAND_2_OFFSET   \
			(VP9_COEF_COUNT_BAND_1_OFFSET + 6*5)
#define VP9_COEF_COUNT_BAND_3_OFFSET   \
			(VP9_COEF_COUNT_BAND_2_OFFSET + 6*5)
#define VP9_COEF_COUNT_BAND_4_OFFSET   \
			(VP9_COEF_COUNT_BAND_3_OFFSET + 6*5)
#define VP9_COEF_COUNT_BAND_5_OFFSET   \
			(VP9_COEF_COUNT_BAND_4_OFFSET + 6*5)
#define VP9_COEF_COUNT_SIZE_ONE_SET    165 /* ((3 +5*6)*5 */
#define VP9_COEF_COUNT_4X4_START       \
	(VP9_COEF_COUNT_START + 0*VP9_COEF_COUNT_SIZE_ONE_SET)
#define VP9_COEF_COUNT_8X8_START       \
	(VP9_COEF_COUNT_START + 4*VP9_COEF_COUNT_SIZE_ONE_SET)
#define VP9_COEF_COUNT_16X16_START     \
	(VP9_COEF_COUNT_START + 8*VP9_COEF_COUNT_SIZE_ONE_SET)
#define VP9_COEF_COUNT_32X32_START     \
	(VP9_COEF_COUNT_START + 12*VP9_COEF_COUNT_SIZE_ONE_SET)
#define VP9_COEF_COUNT_SIZE_PLANE      (2 * VP9_COEF_COUNT_SIZE_ONE_SET)
#define VP9_COEF_COUNT_SIZE            (4 * 2 * 2 * VP9_COEF_COUNT_SIZE_ONE_SET)

#define VP9_INTRA_INTER_COUNT_START    \
	(VP9_COEF_COUNT_START+VP9_COEF_COUNT_SIZE)
#define VP9_INTRA_INTER_COUNT_SIZE     (4*2)
#define VP9_COMP_INTER_COUNT_START     \
	(VP9_INTRA_INTER_COUNT_START+VP9_INTRA_INTER_COUNT_SIZE)
#define VP9_COMP_INTER_COUNT_SIZE      (5*2)
#define VP9_COMP_REF_COUNT_START       \
	(VP9_COMP_INTER_COUNT_START+VP9_COMP_INTER_COUNT_SIZE)
#define VP9_COMP_REF_COUNT_SIZE        (5*2)
#define VP9_SINGLE_REF_COUNT_START     \
	(VP9_COMP_REF_COUNT_START+VP9_COMP_REF_COUNT_SIZE)
#define VP9_SINGLE_REF_COUNT_SIZE      (10*2)
#define VP9_TX_MODE_COUNT_START        \
	(VP9_SINGLE_REF_COUNT_START+VP9_SINGLE_REF_COUNT_SIZE)
#define VP9_TX_MODE_COUNT_SIZE         (12*2)
#define VP9_SKIP_COUNT_START           \
	(VP9_TX_MODE_COUNT_START+VP9_TX_MODE_COUNT_SIZE)
#define VP9_SKIP_COUNT_SIZE            (3*2)
#define VP9_MV_SIGN_0_COUNT_START      \
	(VP9_SKIP_COUNT_START+VP9_SKIP_COUNT_SIZE)
#define VP9_MV_SIGN_0_COUNT_SIZE       (1*2)
#define VP9_MV_SIGN_1_COUNT_START      \
	(VP9_MV_SIGN_0_COUNT_START+VP9_MV_SIGN_0_COUNT_SIZE)
#define VP9_MV_SIGN_1_COUNT_SIZE       (1*2)
#define VP9_MV_BITS_0_COUNT_START      \
	(VP9_MV_SIGN_1_COUNT_START+VP9_MV_SIGN_1_COUNT_SIZE)
#define VP9_MV_BITS_0_COUNT_SIZE       (10*2)
#define VP9_MV_BITS_1_COUNT_START      \
	(VP9_MV_BITS_0_COUNT_START+VP9_MV_BITS_0_COUNT_SIZE)
#define VP9_MV_BITS_1_COUNT_SIZE       (10*2)
#define VP9_MV_CLASS0_HP_0_COUNT_START \
	(VP9_MV_BITS_1_COUNT_START+VP9_MV_BITS_1_COUNT_SIZE)
#define VP9_MV_CLASS0_HP_0_COUNT_SIZE  (2*2)
#define VP9_MV_CLASS0_HP_1_COUNT_START \
	(VP9_MV_CLASS0_HP_0_COUNT_START+VP9_MV_CLASS0_HP_0_COUNT_SIZE)
#define VP9_MV_CLASS0_HP_1_COUNT_SIZE  (2*2)
/* Start merge_tree*/
#define VP9_INTER_MODE_COUNT_START     \
	(VP9_MV_CLASS0_HP_1_COUNT_START+VP9_MV_CLASS0_HP_1_COUNT_SIZE)
#define VP9_INTER_MODE_COUNT_SIZE      (7*4)
#define VP9_IF_Y_MODE_COUNT_START      \
	(VP9_INTER_MODE_COUNT_START+VP9_INTER_MODE_COUNT_SIZE)
#define VP9_IF_Y_MODE_COUNT_SIZE       (10*4)
#define VP9_IF_UV_MODE_COUNT_START     \
	(VP9_IF_Y_MODE_COUNT_START+VP9_IF_Y_MODE_COUNT_SIZE)
#define VP9_IF_UV_MODE_COUNT_SIZE      (10*10)
#define VP9_PARTITION_P_COUNT_START    \
	(VP9_IF_UV_MODE_COUNT_START+VP9_IF_UV_MODE_COUNT_SIZE)
#define VP9_PARTITION_P_COUNT_SIZE     (4*4*4)
#define VP9_INTERP_COUNT_START         \
	(VP9_PARTITION_P_COUNT_START+VP9_PARTITION_P_COUNT_SIZE)
#define VP9_INTERP_COUNT_SIZE          (4*3)
#define VP9_MV_JOINTS_COUNT_START      \
	(VP9_INTERP_COUNT_START+VP9_INTERP_COUNT_SIZE)
#define VP9_MV_JOINTS_COUNT_SIZE       (1 * 4)
#define VP9_MV_CLASSES_0_COUNT_START   \
	(VP9_MV_JOINTS_COUNT_START+VP9_MV_JOINTS_COUNT_SIZE)
#define VP9_MV_CLASSES_0_COUNT_SIZE    (1*11)
#define VP9_MV_CLASS0_0_COUNT_START    \
	(VP9_MV_CLASSES_0_COUNT_START+VP9_MV_CLASSES_0_COUNT_SIZE)
#define VP9_MV_CLASS0_0_COUNT_SIZE     (1*2)
#define VP9_MV_CLASSES_1_COUNT_START   \
	(VP9_MV_CLASS0_0_COUNT_START+VP9_MV_CLASS0_0_COUNT_SIZE)
#define VP9_MV_CLASSES_1_COUNT_SIZE    (1*11)
#define VP9_MV_CLASS0_1_COUNT_START    \
	(VP9_MV_CLASSES_1_COUNT_START+VP9_MV_CLASSES_1_COUNT_SIZE)
#define VP9_MV_CLASS0_1_COUNT_SIZE     (1*2)
#define VP9_MV_CLASS0_FP_0_COUNT_START \
	(VP9_MV_CLASS0_1_COUNT_START+VP9_MV_CLASS0_1_COUNT_SIZE)
#define VP9_MV_CLASS0_FP_0_COUNT_SIZE  (3*4)
#define VP9_MV_CLASS0_FP_1_COUNT_START \
	(VP9_MV_CLASS0_FP_0_COUNT_START+VP9_MV_CLASS0_FP_0_COUNT_SIZE)
#define VP9_MV_CLASS0_FP_1_COUNT_SIZE  (3*4)


#define DC_PRED    0       /* Average of above and left pixels*/
#define V_PRED     1       /* Vertical*/
#define H_PRED     2       /* Horizontal*/
#define D45_PRED   3       /*Directional 45 deg = round(arctan(1/1) * 180/pi)*/
#define D135_PRED  4       /* Directional 135 deg = 180 - 45*/
#define D117_PRED  5       /* Directional 117 deg = 180 - 63*/
#define D153_PRED  6       /* Directional 153 deg = 180 - 27*/
#define D207_PRED  7       /* Directional 207 deg = 180 + 27*/
#define D63_PRED   8       /*Directional 63 deg = round(arctan(2/1) * 180/pi)*/
#define TM_PRED    9       /*True-motion*/

static int clip_prob(int p)
{
	return (p > 255) ? 255 : (p < 1) ? 1 : p;
}

#define ROUND_POWER_OF_TWO(value, n) \
	(((value) + (1 << ((n) - 1))) >> (n))

#define MODE_MV_COUNT_SAT 20
static const int count_to_update_factor[MODE_MV_COUNT_SAT + 1] = {
	0, 6, 12, 19, 25, 32, 38, 44, 51, 57, 64,
	70, 76, 83, 89, 96, 102, 108, 115, 121, 128
};

static void vp9_tree_merge_probs(unsigned int *prev_prob, unsigned int *cur_prob,
	int coef_node_start, int tree_left, int tree_right, int tree_i,
	int node) {

	int prob_32, prob_res, prob_shift;
	int pre_prob, new_prob;
	int den, m_count, get_prob, factor;

	prob_32 = prev_prob[coef_node_start / 4 * 2];
	prob_res = coef_node_start & 3;
	prob_shift = prob_res * 8;
	pre_prob = (prob_32 >> prob_shift) & 0xff;

	den = tree_left + tree_right;

	if (den == 0)
		new_prob = pre_prob;
	else {
		m_count = (den < MODE_MV_COUNT_SAT) ?
			den : MODE_MV_COUNT_SAT;
		get_prob = clip_prob(
				div_r32(((int64_t)tree_left * 256 + (den >> 1)),
				den));
		/*weighted_prob*/
		factor = count_to_update_factor[m_count];
		new_prob = ROUND_POWER_OF_TWO(pre_prob * (256 - factor)
				+ get_prob * factor, 8);
	}
	cur_prob[coef_node_start / 4 * 2] = (cur_prob[coef_node_start / 4 * 2]
			& (~(0xff << prob_shift))) | (new_prob << prob_shift);

	
}


void hakios_vp9_adapt_probs(int pic_count, int prev_kf, int cur_kf, int pre_fc,
	unsigned int *prev_prob, unsigned int *cur_prob, unsigned int *count)
{
	/* 80 * 64bits = 0xF00 ( use 0x1000 4K bytes)
	 *unsigned int prev_prob[496*2];
	 *unsigned int cur_prob[496*2];
	 *0x300 * 128bits = 0x3000 (32K Bytes)
	 *unsigned int count[0x300*4];
	 */

	int tx_size, coef_tx_size_start, coef_count_tx_size_start;
	int plane, coef_plane_start, coef_count_plane_start;
	int type, coef_type_start, coef_count_type_start;
	int band, coef_band_start, coef_count_band_start;
	int cxt_num;
	int cxt, coef_cxt_start, coef_count_cxt_start;
	int node, coef_node_start, coef_count_node_start;

	int tree_i, tree_left, tree_right;
	int mvd_i;

	int count_sat = 24;
	/*int update_factor = 112;*/ /*If COEF_MAX_UPDATE_FACTOR_AFTER_KEY,
	 *use 128
	 */
	/* If COEF_MAX_UPDATE_FACTOR_AFTER_KEY, use 128*/
	/*int update_factor = (pic_count == 1) ? 128 : 112;*/
	int update_factor =   cur_kf ? 112 :
			prev_kf ? 128 : 112;

	int prob_32;
	int prob_res;
	int prob_shift;
	int pre_prob;

	int num, den;
	int get_prob;
	int m_count;
	int factor;

	int new_prob;


	/*adapt_coef_probs*/
	for (tx_size = 0; tx_size < 4; tx_size++) {
		coef_tx_size_start = VP9_COEF_START
			+ tx_size * 4 * VP9_COEF_SIZE_ONE_SET;
		coef_count_tx_size_start = VP9_COEF_COUNT_START
			+ tx_size * 4 * VP9_COEF_COUNT_SIZE_ONE_SET;
		coef_plane_start = coef_tx_size_start;
		coef_count_plane_start = coef_count_tx_size_start;
		for (plane = 0; plane < 2; plane++) {
			coef_type_start = coef_plane_start;
			coef_count_type_start = coef_count_plane_start;
			for (type = 0; type < 2; type++) {
				coef_band_start = coef_type_start;
				coef_count_band_start = coef_count_type_start;
				for (band = 0; band < 6; band++) {
					if (band == 0)
						cxt_num = 3;
					else
						cxt_num = 6;
					coef_cxt_start = coef_band_start;
					coef_count_cxt_start =
						coef_count_band_start;
					for (cxt = 0; cxt < cxt_num; cxt++) {
						const int n0 =
						count[coef_count_cxt_start];
						const int n1 =
						count[coef_count_cxt_start + 1];
						const int n2 =
						count[coef_count_cxt_start + 2];
						const int neob =
						count[coef_count_cxt_start + 3];
						const int nneob =
						count[coef_count_cxt_start + 4];
						const unsigned int
						branch_ct[3][2] = {
						{ neob, nneob },
						{ n0, n1 + n2 },
						{ n1, n2 }
						};
						coef_node_start =
							coef_cxt_start;
						for
						(node = 0; node < 3; node++) {
							prob_32 =
							prev_prob[
							coef_node_start
							/ 4 * 2];
							prob_res =
							coef_node_start & 3;
							prob_shift =
							prob_res * 8;
							pre_prob =
							(prob_32 >> prob_shift)
							& 0xff;

							/*get_binary_prob*/
							num =
							branch_ct[node][0];
							den =
							branch_ct[node][0] +
							 branch_ct[node][1];
							m_count = (den <
							count_sat)
							? den : count_sat;

							get_prob =
							(den == 0) ? 128u :
							clip_prob(
							div_r32(((int64_t)
							num * 256
							+ (den >> 1)),
							den));

							factor =
							update_factor * m_count
							/ count_sat;
							new_prob =
							ROUND_POWER_OF_TWO
							(pre_prob *
							(256 - factor) +
							get_prob * factor, 8);

							cur_prob[coef_node_start
							/ 4 * 2] =
							(cur_prob
							[coef_node_start
							/ 4 * 2] & (~(0xff <<
							prob_shift))) |
							(new_prob <<
							prob_shift);

							coef_node_start += 1;
						}

						coef_cxt_start =
							coef_cxt_start + 3;
						coef_count_cxt_start =
							coef_count_cxt_start
							+ 5;
					}
					if (band == 0) {
						coef_band_start += 10;
						coef_count_band_start += 15;
					} else {
						coef_band_start += 18;
						coef_count_band_start += 30;
					}
				}
				coef_type_start += VP9_COEF_SIZE_ONE_SET;
				coef_count_type_start +=
					VP9_COEF_COUNT_SIZE_ONE_SET;
			}
			coef_plane_start += 2 * VP9_COEF_SIZE_ONE_SET;
			coef_count_plane_start +=
					2 * VP9_COEF_COUNT_SIZE_ONE_SET;
		}
	}

	if (cur_kf == 0) {
		/*mode_mv_merge_probs - merge_intra_inter_prob*/
		for (coef_count_node_start = VP9_INTRA_INTER_COUNT_START;
		coef_count_node_start < (VP9_MV_CLASS0_HP_1_COUNT_START +
		VP9_MV_CLASS0_HP_1_COUNT_SIZE);	coef_count_node_start += 2) {

			if (coef_count_node_start ==
				VP9_INTRA_INTER_COUNT_START) {
				coef_node_start = VP9_INTRA_INTER_START;
			} else if (coef_count_node_start ==
				VP9_COMP_INTER_COUNT_START) {
				coef_node_start = VP9_COMP_INTER_START;
			}
			/*
			 *else if (coef_count_node_start ==
			 *	VP9_COMP_REF_COUNT_START) {
			 *	pr_info(" # merge_comp_inter_prob\n");
			 *	coef_node_start = VP9_COMP_REF_START;
			 *}
			 *else if (coef_count_node_start ==
			 *	VP9_SINGLE_REF_COUNT_START) {
			 *	pr_info(" # merge_comp_inter_prob\n");
			 *	coef_node_start = VP9_SINGLE_REF_START;
			 *}
			 */
			else if (coef_count_node_start ==
				VP9_TX_MODE_COUNT_START) {
				coef_node_start = VP9_TX_MODE_START;
			} else if (coef_count_node_start ==
				VP9_SKIP_COUNT_START) {
				coef_node_start = VP9_SKIP_START;
			} else if (coef_count_node_start ==
				VP9_MV_SIGN_0_COUNT_START) {
				coef_node_start = VP9_MV_SIGN_0_START;
			} else if (coef_count_node_start ==
				VP9_MV_SIGN_1_COUNT_START) {
				coef_node_start = VP9_MV_SIGN_1_START;
			} else if (coef_count_node_start ==
				VP9_MV_BITS_0_COUNT_START) {
				coef_node_start = VP9_MV_BITS_0_START;
			} else if (coef_count_node_start ==
				VP9_MV_BITS_1_COUNT_START) {
				coef_node_start = VP9_MV_BITS_1_START;
			} else if (coef_count_node_start ==
					VP9_MV_CLASS0_HP_0_COUNT_START) {
				coef_node_start = VP9_MV_CLASS0_HP_0_START;
			}


		den = count[coef_count_node_start] +
			count[coef_count_node_start + 1];

		prob_32 = prev_prob[coef_node_start / 4 * 2];
		prob_res = coef_node_start & 3;
		prob_shift = prob_res * 8;
		pre_prob = (prob_32 >> prob_shift) & 0xff;

		if (den == 0)
			new_prob = pre_prob;
		else {
			m_count = (den < MODE_MV_COUNT_SAT) ?
				den : MODE_MV_COUNT_SAT;
			get_prob =
				clip_prob(
				div_r32(((int64_t)count[coef_count_node_start]
				* 256 + (den >> 1)),
				den));
			/*weighted_prob*/
			factor = count_to_update_factor[m_count];
			new_prob =
				ROUND_POWER_OF_TWO(pre_prob * (256 - factor)
				+ get_prob * factor, 8);
		}
		cur_prob[coef_node_start / 4 * 2] =
			(cur_prob[coef_node_start / 4 * 2] &
			(~(0xff << prob_shift)))
			| (new_prob << prob_shift);

		coef_node_start = coef_node_start + 1;
	}
	coef_node_start = VP9_INTER_MODE_START;
	coef_count_node_start = VP9_INTER_MODE_COUNT_START;
	for (tree_i = 0; tree_i < 7; tree_i++) {
		for (node = 0; node < 3; node++) {
			switch (node) {
			case 2:
				tree_left =
				count[coef_count_node_start + 1];
				tree_right =
				count[coef_count_node_start + 3];
				break;
			case 1:
				tree_left =
				count[coef_count_node_start + 0];
				tree_right =
				count[coef_count_node_start + 1]
				+ count[coef_count_node_start + 3];
				break;
			default:
				tree_left =
				count[coef_count_node_start + 2];
				tree_right =
				count[coef_count_node_start + 0]
				+ count[coef_count_node_start + 1]
				+ count[coef_count_node_start + 3];
				break;

			}

			vp9_tree_merge_probs(prev_prob, cur_prob,
				coef_node_start, tree_left, tree_right,
				tree_i, node);

			coef_node_start = coef_node_start + 1;
		}
		coef_count_node_start = coef_count_node_start + 4;
	}
	coef_node_start = VP9_IF_Y_MODE_START;
	coef_count_node_start = VP9_IF_Y_MODE_COUNT_START;
	for (tree_i = 0; tree_i < 14; tree_i++) {
		for (node = 0; node < 9; node++) {
			switch (node) {
			case 8:
				tree_left =
				count[coef_count_node_start+D153_PRED];
				tree_right =
				count[coef_count_node_start+D207_PRED];
				break;
			case 7:
				tree_left =
				count[coef_count_node_start+D63_PRED];
				tree_right =
				count[coef_count_node_start+D207_PRED] +
				count[coef_count_node_start+D153_PRED];
				break;
			case 6:
				tree_left =
				count[coef_count_node_start + D45_PRED];
				tree_right =
				count[coef_count_node_start+D207_PRED] +
				count[coef_count_node_start+D153_PRED] +
				count[coef_count_node_start+D63_PRED];
				break;
			case 5:
				tree_left =
				count[coef_count_node_start+D135_PRED];
				tree_right =
				count[coef_count_node_start+D117_PRED];
				break;
			case 4:
				tree_left =
				count[coef_count_node_start+H_PRED];
				tree_right =
				count[coef_count_node_start+D117_PRED] +
				count[coef_count_node_start+D135_PRED];
				break;
			case 3:
				tree_left =
				count[coef_count_node_start+H_PRED] +
				count[coef_count_node_start+D117_PRED] +
				count[coef_count_node_start+D135_PRED];
				tree_right =
				count[coef_count_node_start+D45_PRED] +
				count[coef_count_node_start+D207_PRED] +
				count[coef_count_node_start+D153_PRED] +
				count[coef_count_node_start+D63_PRED];
				break;
			case 2:
				tree_left =
				count[coef_count_node_start+V_PRED];
				tree_right =
				count[coef_count_node_start+H_PRED] +
				count[coef_count_node_start+D117_PRED] +
				count[coef_count_node_start+D135_PRED] +
				count[coef_count_node_start+D45_PRED] +
				count[coef_count_node_start+D207_PRED] +
				count[coef_count_node_start+D153_PRED] +
				count[coef_count_node_start+D63_PRED];
				break;
			case 1:
				tree_left =
				count[coef_count_node_start+TM_PRED];
				tree_right =
				count[coef_count_node_start+V_PRED] +
				count[coef_count_node_start+H_PRED] +
				count[coef_count_node_start+D117_PRED] +
				count[coef_count_node_start+D135_PRED] +
				count[coef_count_node_start+D45_PRED] +
				count[coef_count_node_start+D207_PRED] +
				count[coef_count_node_start+D153_PRED] +
				count[coef_count_node_start+D63_PRED];
				break;
			default:
				tree_left =
				count[coef_count_node_start+DC_PRED];
				tree_right =
				count[coef_count_node_start+TM_PRED] +
				count[coef_count_node_start+V_PRED] +
				count[coef_count_node_start+H_PRED] +
				count[coef_count_node_start+D117_PRED] +
				count[coef_count_node_start+D135_PRED] +
				count[coef_count_node_start+D45_PRED] +
				count[coef_count_node_start+D207_PRED] +
				count[coef_count_node_start+D153_PRED] +
				count[coef_count_node_start+D63_PRED];
				break;

				}

			vp9_tree_merge_probs(prev_prob, cur_prob,
				coef_node_start, tree_left, tree_right,
				tree_i, node);

			coef_node_start = coef_node_start + 1;
		}
		coef_count_node_start = coef_count_node_start + 10;
	}

	coef_node_start = VP9_PARTITION_P_START;
	coef_count_node_start = VP9_PARTITION_P_COUNT_START;
	for (tree_i = 0; tree_i < 16; tree_i++) {
		for (node = 0; node < 3; node++) {
			switch (node) {
			case 2:
				tree_left =
				count[coef_count_node_start + 2];
				tree_right =
				count[coef_count_node_start + 3];
				break;
			case 1:
				tree_left =
				count[coef_count_node_start + 1];
				tree_right =
				count[coef_count_node_start + 2] +
				count[coef_count_node_start + 3];
				break;
			default:
				tree_left =
				count[coef_count_node_start + 0];
				tree_right =
				count[coef_count_node_start + 1] +
				count[coef_count_node_start + 2] +
				count[coef_count_node_start + 3];
				break;

			}

			vp9_tree_merge_probs(prev_prob, cur_prob,
				coef_node_start,
				tree_left, tree_right, tree_i, node);

			coef_node_start = coef_node_start + 1;
		}
		coef_count_node_start = coef_count_node_start + 4;
	}

	coef_node_start = VP9_INTERP_START;
	coef_count_node_start = VP9_INTERP_COUNT_START;
	for (tree_i = 0; tree_i < 4; tree_i++) {
		for (node = 0; node < 2; node++) {
			switch (node) {
			case 1:
				tree_left =
				count[coef_count_node_start + 1];
				tree_right =
				count[coef_count_node_start + 2];
				break;
			default:
				tree_left =
				count[coef_count_node_start + 0];
				tree_right =
				count[coef_count_node_start + 1] +
				count[coef_count_node_start + 2];
				break;

			}

			vp9_tree_merge_probs(prev_prob, cur_prob,
				coef_node_start,
				tree_left, tree_right, tree_i, node);

			coef_node_start = coef_node_start + 1;
		}
		coef_count_node_start = coef_count_node_start + 3;
	}

	coef_node_start = VP9_MV_JOINTS_START;
	coef_count_node_start = VP9_MV_JOINTS_COUNT_START;
	for (tree_i = 0; tree_i < 1; tree_i++) {
		for (node = 0; node < 3; node++) {
			switch (node) {
			case 2:
				tree_left =
				count[coef_count_node_start + 2];
				tree_right =
				count[coef_count_node_start + 3];
				break;
			case 1:
				tree_left =
				count[coef_count_node_start + 1];
				tree_right =
				count[coef_count_node_start + 2] +
				count[coef_count_node_start + 3];
				break;
			default:
				tree_left =
				count[coef_count_node_start + 0];
				tree_right =
				count[coef_count_node_start + 1] +
				count[coef_count_node_start + 2] +
				count[coef_count_node_start + 3];
				break;
			}

			vp9_tree_merge_probs(prev_prob, cur_prob,
				coef_node_start,
				tree_left, tree_right, tree_i, node);

			coef_node_start = coef_node_start + 1;
		}
		coef_count_node_start = coef_count_node_start + 4;
	}

	for (mvd_i = 0; mvd_i < 2; mvd_i++) {
		coef_node_start =
			mvd_i ? VP9_MV_CLASSES_1_START : VP9_MV_CLASSES_0_START;
		coef_count_node_start =
			mvd_i ? VP9_MV_CLASSES_1_COUNT_START
			: VP9_MV_CLASSES_0_COUNT_START;
		tree_i = 0;
		for (node = 0; node < 10; node++) {
			switch (node) {
			case 9:
				tree_left =
				count[coef_count_node_start + 9];
				tree_right =
				count[coef_count_node_start + 10];
				break;
			case 8:
				tree_left =
				count[coef_count_node_start + 7];
				tree_right =
				count[coef_count_node_start + 8];
				break;
			case 7:
				tree_left =
				count[coef_count_node_start + 7] +
				count[coef_count_node_start + 8];
				tree_right =
				count[coef_count_node_start + 9] +
				count[coef_count_node_start + 10];
				break;
			case 6:
				tree_left =
				count[coef_count_node_start + 6];
				tree_right =
				count[coef_count_node_start + 7] +
				count[coef_count_node_start + 8] +
				count[coef_count_node_start + 9] +
				count[coef_count_node_start + 10];
				break;
			case 5:
				tree_left =
				count[coef_count_node_start + 4];
				tree_right =
				count[coef_count_node_start + 5];
				break;
			case 4:
				tree_left =
				count[coef_count_node_start + 4] +
				count[coef_count_node_start + 5];
				tree_right =
				count[coef_count_node_start + 6] +
				count[coef_count_node_start + 7] +
				count[coef_count_node_start + 8] +
				count[coef_count_node_start + 9] +
				count[coef_count_node_start + 10];
				break;
			case 3:
				tree_left =
				count[coef_count_node_start + 2];
				tree_right =
				count[coef_count_node_start + 3];
				break;
			case 2:
				tree_left =
				count[coef_count_node_start + 2] +
				count[coef_count_node_start + 3];
				tree_right =
				count[coef_count_node_start + 4] +
				count[coef_count_node_start + 5] +
				count[coef_count_node_start + 6] +
				count[coef_count_node_start + 7] +
				count[coef_count_node_start + 8] +
				count[coef_count_node_start + 9] +
				count[coef_count_node_start + 10];
				break;
			case 1:
				tree_left =
				count[coef_count_node_start + 1];
				tree_right =
				count[coef_count_node_start + 2] +
				count[coef_count_node_start + 3] +
				count[coef_count_node_start + 4] +
				count[coef_count_node_start + 5] +
				count[coef_count_node_start + 6] +
				count[coef_count_node_start + 7] +
				count[coef_count_node_start + 8] +
				count[coef_count_node_start + 9] +
				count[coef_count_node_start + 10];
				break;
			default:
				tree_left =
				count[coef_count_node_start + 0];
				tree_right =
				count[coef_count_node_start + 1] +
				count[coef_count_node_start + 2] +
				count[coef_count_node_start + 3] +
				count[coef_count_node_start + 4] +
				count[coef_count_node_start + 5] +
				count[coef_count_node_start + 6] +
				count[coef_count_node_start + 7] +
				count[coef_count_node_start + 8] +
				count[coef_count_node_start + 9] +
				count[coef_count_node_start + 10];
				break;

			}

			vp9_tree_merge_probs(prev_prob, cur_prob,
				coef_node_start, tree_left, tree_right,
				tree_i, node);

			coef_node_start = coef_node_start + 1;
		}

		coef_node_start =
			mvd_i ? VP9_MV_CLASS0_1_START : VP9_MV_CLASS0_0_START;
		coef_count_node_start =
			mvd_i ? VP9_MV_CLASS0_1_COUNT_START :
			VP9_MV_CLASS0_0_COUNT_START;
		tree_i = 0;
		node = 0;
		tree_left = count[coef_count_node_start + 0];
		tree_right = count[coef_count_node_start + 1];

		vp9_tree_merge_probs(prev_prob, cur_prob, coef_node_start,
			tree_left, tree_right, tree_i, node);
		coef_node_start =
			mvd_i ? VP9_MV_CLASS0_FP_1_START :
			VP9_MV_CLASS0_FP_0_START;
		coef_count_node_start =
			mvd_i ? VP9_MV_CLASS0_FP_1_COUNT_START :
			VP9_MV_CLASS0_FP_0_COUNT_START;
		for (tree_i = 0; tree_i < 3; tree_i++) {
			for (node = 0; node < 3; node++) {
				switch (node) {
				case 2:
					tree_left =
					count[coef_count_node_start + 2];
					tree_right =
					count[coef_count_node_start + 3];
					break;
				case 1:
					tree_left =
					count[coef_count_node_start + 1];
					tree_right =
					count[coef_count_node_start + 2]
					+ count[coef_count_node_start + 3];
					break;
				default:
					tree_left =
					count[coef_count_node_start + 0];
					tree_right =
					count[coef_count_node_start + 1]
					+ count[coef_count_node_start + 2]
					+ count[coef_count_node_start + 3];
					break;

				}

				vp9_tree_merge_probs(prev_prob, cur_prob,
					coef_node_start, tree_left, tree_right,
					tree_i, node);

				coef_node_start = coef_node_start + 1;
			}
			coef_count_node_start = coef_count_node_start + 4;
		}

	} /* for mvd_i (mvd_y or mvd_x)*/
}

}

