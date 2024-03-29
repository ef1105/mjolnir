/**
 * @file namcos21.h
 */
READ16_HANDLER(namcos21_video_enable_r);
WRITE16_HANDLER(namcos21_video_enable_w);

#define NAMCOS21_POLY_FRAME_WIDTH 496
#define NAMCOS21_POLY_FRAME_HEIGHT 480

extern void namcos21_ClearPolyFrameBuffer( void );
extern void namcos21_DrawQuad( int sx[4], int sy[4], int zcode[4], int color );

extern READ16_HANDLER(winrun_gpu_color_r);
extern WRITE16_HANDLER(winrun_gpu_color_w);

extern READ16_HANDLER(winrun_gpu_videoram_r);
extern WRITE16_HANDLER(winrun_gpu_videoram_w);

extern READ16_HANDLER(winrun_gpu_register_r);
extern WRITE16_HANDLER(winrun_gpu_register_w);

extern VIDEO_UPDATE( namcos21 );
