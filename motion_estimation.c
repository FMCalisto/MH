/* Computer Organization (OC)                                             */
/* Architectures for Embedded Computing (ACE)                             */
/* Lab: Caches                                                            */
/* Nuno.Roma@ist.utl.pt                                                   */
/* IST, Lisbon-Portugal, 2009-09-15                                       */
/* nuno.m.santos@ist.utl.pt                                               */
/* IST, Lisbon-Portugal, 2013-11-08                                       */

/* The following program implements the Full-Search Block-Matching (FSBM) */
/* Motion-Estimation (ME) algorithm for a given YUV video sequence,       */
/* registering all the addresses that are accessed from the considered    */
/* frame memory in an output trace-file.                                  */

#include <stdio.h>
#include <string.h>   // memset
#include <limits.h>   // INT_MAX

/**************************************************************************/
/* Simulator Parametrization                                              */
/**************************************************************************/
// number of frames to process
#define n_frames 1

/**************************************************************************/
/* Frame Dimension Parametrization                                        */
/**************************************************************************/
// number of MB lines
#define line_max 9		// QCIF
// number of MB columns
#define col_max 11		// QCIF
// number of luminance pixels
#define pixelsperframe (M * M * line_max * col_max)

/**************************************************************************/
/* Motion Estimation Algorithm Parametrization                            */
/**************************************************************************/
// size of each macroblock M*M
#define M 16
//maximum displacement within the search area (2p+M-1)*(2p+M-1)
#define p 8

/**************************************************************************/
/* Input and Output Files                                                 */
/**************************************************************************/
// Input video file
#define VIDEO_FILE "table_tennis_qcif_3frames.yuv"
// Motion estimation algorithm output file
#define RES_FILE   "results.log"
// Frame-Memory addresses output trace file
#define TRACE_FILE "trace.log"

/**************************************************************************/
/* Auxiliary constants and macros                                         */
/**************************************************************************/
#define PREVIOUS_Y  0
#define PREVIOUS_CB 1
#define PREVIOUS_CR 2
#define CURRENT_Y   3
#define CURRENT_CB  4
#define CURRENT_CR  5

#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))


typedef struct _MV {
    char x;
    char y;
    unsigned short sad;
} MV;

typedef struct _COORD {
    int l;
    int c;
} COORD;

typedef struct _WINDOW {
    COORD UL;
    COORD LR;
    COORD CENTER;
} WINDOW;


/**************************************************************************/
/* Grobal variables                                                       */
/**************************************************************************/
unsigned char frame_memory[3 * pixelsperframe];
FILE *trace_file;


/**************************************************************************/
/* Frame-Memory trace auxiliary functions                                 */
/**************************************************************************/

/**************************************************************************/
/* frame_memory_read - reads a frame-memory memory position and registers */
/*                     the corresponding address in the trace-file.       */
/**************************************************************************/
unsigned char frame_memory_read(int frame, unsigned char *frame_memory, int relative_address)
{
    int offset, absolute_address;

    switch(frame){
	case PREVIOUS_Y:
	    offset = 0.00 * pixelsperframe;
	    break;
	case PREVIOUS_CB:
	    offset = 1.00 * pixelsperframe;
	    break;
	case PREVIOUS_CR:
	    offset = 1.25 * pixelsperframe;
	    break;
	case CURRENT_Y:
	    offset = 1.50 * pixelsperframe;
	    break;
	case CURRENT_CB:
	    offset = 2.50 * pixelsperframe;
	    break;
	case CURRENT_CR:
	    offset = 2.75 * pixelsperframe;
	    break;
	default:
	    printf("ERROR: Frame type not defined!\n");
	    break;
    }
    absolute_address = offset + relative_address;
    fprintf(trace_file,"%c %08x %d\n",'r',absolute_address,1);
    return frame_memory[absolute_address];
}

/**************************************************************************/
/* frame_memory_write - writes a frame-memory memory position and         */
/*                      registers the corresponding address in the        */
/*                      trace-file.                                       */
/**************************************************************************/
unsigned char frame_memory_write(int frame, unsigned char *frame_memory, int relative_address,unsigned char pixel)
{
    int offset, absolute_address;

    switch(frame){
	case PREVIOUS_Y:
	    offset = 0.00 * pixelsperframe;
	    break;
	case PREVIOUS_CB:
	    offset = 1.00 * pixelsperframe;
	    break;
	case PREVIOUS_CR:
	    offset = 1.25 * pixelsperframe;
	    break;
	case CURRENT_Y:
	    offset = 1.50 * pixelsperframe;
	    break;
	case CURRENT_CB:
	    offset = 2.50 * pixelsperframe;
	    break;
	case CURRENT_CR:
	    offset = 2.75 * pixelsperframe;
	    break;
	default:
	    printf("ERROR: Frame type not defined!\n");
	    break;
    }
    absolute_address = offset + relative_address;
    fprintf(trace_file,"%c %08x %d\n",'w',absolute_address,1);
    return frame_memory[absolute_address]=pixel;
}

/**************************************************************************/
/* get_image - reads all the luminance (Y) and chrominance (Cb & Cr)      */
/*             pixels from the input video file.                          */
/**************************************************************************/
void get_image(int frame, unsigned char *image, FILE * fp)
{
    int i, size;

    switch(frame){
	case PREVIOUS_Y:
	case CURRENT_Y:
	    size = 1.00 * pixelsperframe;
	    break;
	case PREVIOUS_CB:
	case PREVIOUS_CR:
	case CURRENT_CB:
	case CURRENT_CR:
	    size = 0.25 * pixelsperframe;
	    break;
	default:
	    printf("ERROR: Frame type not defined!\n");
	    break;
    }
    for (i = 0; i < size; i++)
        frame_memory_write(frame, frame_memory, i, (unsigned char) fgetc(fp));
}

/**************************************************************************/
/* Main function                                                          */
/**************************************************************************/
int main( /*int argc, char **argv */ )
{
    // File pointers
    FILE *fp1, *fp2;

    // Output MVs tables
    MV SOFT_MVs[line_max][col_max];
    int x_SAD_min;
    int y_SAD_min;
 
    WINDOW SA;
    WINDOW RB;

    int rb_pixel, sa_pixel;
    int diff;
    int SAD_min;
    int SAD_tmp;

    int block_l, block_c;
    int l_candidate, c_candidate;
    int l_intrablock, c_intrablock;

    short i;

    fp1 = (FILE *) fopen(VIDEO_FILE, "rb");
    fp2 = (FILE *) fopen(RES_FILE, "w");
    trace_file = (FILE *) fopen(TRACE_FILE, "w");

    // Frame no.0 (INTRA) will be the reference frame for frame no.1
    get_image(PREVIOUS_Y,  frame_memory, fp1);
    get_image(PREVIOUS_CB, frame_memory, fp1);
    get_image(PREVIOUS_CR, frame_memory, fp1);
    // Processing of the remaining frames (INTER)
    for (i = 1; i <= n_frames; i++) {
	fprintf(fp2, "\n************************************ FRAME No.%2d ************************************\n", i);
	// MV and SAD tables initialization
	memset(SOFT_MVs, 0, sizeof(SOFT_MVs));
	// Input of the current image
	get_image(CURRENT_Y,  frame_memory, fp1);
	get_image(CURRENT_CB, frame_memory, fp1);
	get_image(CURRENT_CR, frame_memory, fp1);

	// MB processing
	for (block_l = 0; block_l < line_max; block_l++) {
	    for (block_c = 0; block_c < col_max; block_c++) {

                RB.UL.l=block_l*M;
                RB.UL.c=block_c*M;
                RB.LR.l=block_l*M+(M-1);
                RB.LR.c=block_c*M+(M-1);
                SA.UL.l=MAX(0,block_l*M-p);
                SA.UL.c=MAX(0,block_c*M-p);
                SA.LR.l=MIN(block_l*M+(M-1)+p,line_max*M);
                SA.LR.c=MIN(block_c*M+(M-1)+p,col_max*M);
                SA.CENTER.l=block_l*M;
                SA.CENTER.c=block_c*M;

		SAD_min = INT_MAX;

		for (l_candidate = SA.UL.l; l_candidate <= SA.LR.l-(M-1); l_candidate++) {
		    for (c_candidate = SA.UL.c; c_candidate <= SA.LR.c-(M-1); c_candidate++) {
			SAD_tmp = 0;
			for (l_intrablock = 0; l_intrablock < M; l_intrablock++) {
			    for (c_intrablock = 0; c_intrablock < M; c_intrablock++) {
				rb_pixel = frame_memory_read(  CURRENT_Y, frame_memory, \
							       (RB.UL.l+l_intrablock)*col_max*M + (RB.UL.c+c_intrablock));
				sa_pixel = frame_memory_read(  PREVIOUS_Y, frame_memory, \
							       (l_candidate+l_intrablock)*col_max*M + c_candidate + c_intrablock);
				diff = rb_pixel - sa_pixel;
				SAD_tmp += (diff > 0) ? (diff) : (-diff);
			    }
			}
			if (SAD_tmp <= SAD_min) {
			    SAD_min = SAD_tmp;
			    x_SAD_min = c_candidate;
			    y_SAD_min = l_candidate;
			}
		    }
		}
		SOFT_MVs[block_l][block_c].sad = SAD_min;
		SOFT_MVs[block_l][block_c].x = x_SAD_min - SA.CENTER.c;
		SOFT_MVs[block_l][block_c].y = y_SAD_min - SA.CENTER.l;
	    }
	}
	// Output results
	fprintf(fp2, "MVs Table (MV_s,MV_y):\n");
	for (block_l = 0; block_l < line_max; block_l++) {
	    for (block_c = 0; block_c < col_max; block_c++) {
		fprintf(fp2, "(%d,%d)\t", (SOFT_MVs[block_l][block_c]).x, (SOFT_MVs[block_l][block_c]).y);
		if (block_c == col_max - 1)
		    fprintf(fp2, "\n");
	    }
	}
	fprintf(fp2, "SAD_MB Table:\n");
	for (block_l = 0; block_l < line_max; block_l++) {
	    for (block_c = 0; block_c < col_max; block_c++) {
		fprintf(fp2, "%d\t", (SOFT_MVs[block_l][block_c]).sad);
		if (block_c == col_max - 1)
		    fprintf(fp2, "\n");
	    }
	}
	// Sets the current image as the reference image for the next frame
	for (i=0;i<pixelsperframe;i++){
	    frame_memory_write(PREVIOUS_Y, frame_memory, i, frame_memory_read(CURRENT_Y, frame_memory, i) );
	}
	for (i=0;i<0.25*pixelsperframe;i++){
	    frame_memory_write(PREVIOUS_CB, frame_memory, i, frame_memory_read(CURRENT_CB, frame_memory, i) );
	    frame_memory_write(PREVIOUS_CR, frame_memory, i, frame_memory_read(CURRENT_CR, frame_memory, i) );
	}
    }
    fclose(fp1);
    fclose(fp2);
    printf("***************************************************\n");
    printf("Done! Simulation results have been successfully written to file: %s.\n\n\n", RES_FILE);

    return (0);
}
