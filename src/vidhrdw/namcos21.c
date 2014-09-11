/**
 * Namco System 21 Video Hardware
todo:
- solvalou: sprite transp
- solvalou: gradient fix
- solvalou: bad 3d

With all of that out of the way, when it works, it generally works quite well. To save the state of a game, use the default save state sequence of Shift + F7. At this point, you will get a prompt asking you to select a slot. Type a letter from 'a' to 'z' and the game's state will be saved. If the game doesn't officially support save states, you will be warned at this time.

To restore a saved state, use the default load state key F7. Again, you will prompted to select a slot to load from, and the state will be loaded. If something has changed in the way the driver or any of its dependent code handles save states, then the restore may fail, so be warned. 

complete save state for namcoic (partial)

 fix near-plane clipping (emulate dsp)
 fix ddraw horizon
 viewport culling
 backface culling
 */
#include "driver.h"
#include "vidhrdw/generic.h"
#include "namcos2.h"
#include "namcoic.h"
#include "namcos21.h"
#include "poly3d.h"
#include "state.h"

static int mbReady;
extern int winrun_irq_enable;
static const INT32 *namcos21_pointrom;
extern UINT8 *namcos21_pointram;
void namcos21_DrawPolygons( void );

extern UINT16 mSpritePos[4];
static int mPalXOR = 0xf;

#define MAX_QUADS 8000
static int mSkyQuads[2];
static int mNumQuads[2];
static int mMaxQuads;

struct QuadInfo
{
	int clip;
	int sx[4];
	int sy[4];
	int zcode[4];
	int color;
} mQuadInfo[2][MAX_QUADS];

static int mQuadBank;

struct QuadInfo *
NewQuad( void )
{
	int count = mNumQuads[mQuadBank];
	if( count<MAX_QUADS )
	{
		mNumQuads[mQuadBank] = count+1;
		return &mQuadInfo[mQuadBank][count];
	}
	printf( "too-many quads!\n" );
	exit(1);
	return NULL;
}

void
namcos21_ClearPolyFrameBuffer( void )
{
	mQuadBank = 1-mQuadBank;
	mNumQuads[mQuadBank] = 0;
	mSkyQuads[mQuadBank] = 0;
}

void
namcos21_DrawQuad( int sx[4], int sy[4], int zcode[4], int color )
{
	struct QuadInfo *quad = NewQuad();
	int i;
	for( i=0; i<4; i++ )
	{
		quad->sx[i] = sx[i];
		quad->sy[i] = sy[i];
		quad->zcode[i] = zcode[i];
	}
	quad->color = 0x4000|(color&0xff);
} /* namcos21_DrawQuad */

static TileCache *mpSpriteTileCache;

static void
GetSpriteTileData( int tileNumber, int color, unsigned char *pBuf, void *pUserParam )
{
   const gfx_element *gfx = Machine->gfx[0];
   const UINT8 *pPenData = gfx->gfxdata + (tileNumber%gfx->total_elements)*gfx->char_modulo;
	const pen_t *pPal = &gfx->colortable[gfx->color_granularity *(color%gfx->total_colors)];
   int x,y;
	for( y=0; y<16; y++ )
   {
      for( x=0; x<16; x++ )
      {
         int pen = *pPenData++;
			UINT8 r,g,b;
			switch( pen )
			{
			case 0xff:
            *pBuf++ = 0x00;
            *pBuf++ = 0x00;
            *pBuf++ = 0x00;
            *pBuf++ = 0x00; /* wholly transparent */
				break;
			case 0x00: /* solvalou - exhaust */
				*pBuf++ = 0xff;
				*pBuf++ = 0xff;
				*pBuf++ = 0x00;
				*pBuf++ = 0x80;
				break;
			case 0x01: /* darken */
				*pBuf++ = 0x00;
				*pBuf++ = 0x00;
				*pBuf++ = 0x00;
				*pBuf++ = 0x80;
				break;
			default:
				palette_get_color(pPal[pen],&r,&g,&b);
            *pBuf++ = r;
            *pBuf++ = g;
            *pBuf++ = b;
            *pBuf++ = 0xff; /* opaque */
				break;
         }
      }
   }
} /* GetSpriteTileData */

/**
 * 0x00000 sprite attr (page0)
 * 0x02000 sprite list (page0)
 *
 * 0x02400 window attributes
 * 0x04000 format
 * 0x08000 tile
 * 0x10000 sprite attr (page1)
 * 0x14000 sprite list (page1)
 */
static void
DrawSpriteC355( const rectangle *cliprect, const UINT16 *pSource, int pri, int zpos )
{
	unsigned screen_height_remaining, screen_width_remaining;
	unsigned source_height_remaining, source_width_remaining;
	int hpos,vpos;
	UINT16 hsize,vsize;
	UINT16 palette;
	UINT16 linkno;
	UINT16 offset;
	UINT16 format;
	int tile_index;
	int num_cols,num_rows;
	int dx,dy;
	int row,col;
	int sx,sy,tile;
	int flipx,flipy;
	UINT32 zoomx, zoomy;
	int tile_screen_width;
	int tile_screen_height;
	const UINT16 *spriteformat16 = &spriteram16[0x4000/2];
	const UINT16 *spritetile16   = &spriteram16[0x8000/2];
	int color;
	const UINT16 *pWinAttr;
	rectangle clip;
	int xscroll, yscroll;

	/**
     * ----xxxx-------- window select
     * --------xxxx---- priority
     * ------------xxxx palette select
     */
	palette = pSource[6];
	{
		int sprite_pri = (palette>>4)&0xf;
		if( pri==0 )
		{
			if( sprite_pri>=3 ) return;
		}
		else
		{
			if( sprite_pri<3 ) return;
		}
	}

	linkno	= pSource[0]; /* LINKNO */
	offset	= pSource[1]; /* OFFSET */
	hpos		= pSource[2]; /* HPOS       0x000..0x7ff (signed) */
	vpos		= pSource[3]; /* VPOS       0x000..0x7ff (signed) */
	hsize		= pSource[4]; /* HSIZE      max 0x3ff pixels */
	vsize		= pSource[5]; /* VSIZE      max 0x3ff pixels */
	/* pSource[6] contains priority/palette */
	/* pSource[7] is used in Lucky & Wild, possibly for sprite-road priority */

	if( linkno*4>=0x4000/2 ) return; /* avoid garbage memory reads */

	xscroll = (INT16)mSpritePos[1];
	yscroll = (INT16)mSpritePos[0];

//	xscroll &= 0x1ff; if( xscroll & 0x100 ) xscroll |= ~0x1ff;
	yscroll &= 0x1ff; if( yscroll & 0x100 ) yscroll |= ~0x1ff;

//	if( bitmap->width > 288 )
//	{ /* Medium Resolution: System21 adjust */
	xscroll &= 0x3ff; if( xscroll & 0x200 ) xscroll |= ~0x3ff;
			yscroll += 0x20;
//	}

	hpos -= xscroll;
	vpos -= yscroll;
	pWinAttr = &spriteram16[0x2400/2+((palette>>8)&0xf)*4];
	clip.min_x = pWinAttr[0] - xscroll;
	clip.max_x = pWinAttr[1] - xscroll;
	clip.min_y = pWinAttr[2] - yscroll;
	clip.max_y = pWinAttr[3] - yscroll;
	if( clip.min_x < cliprect->min_x ){ clip.min_x = cliprect->min_x; }
	if( clip.min_y < cliprect->min_y ){ clip.min_y = cliprect->min_y; }
	if( clip.max_x > cliprect->max_x ){ clip.max_x = cliprect->max_x; }
	if( clip.max_y > cliprect->max_y ){ clip.max_y = cliprect->max_y; }
	hpos&=0x7ff; if( hpos&0x400 ) hpos |= ~0x7ff; /* sign extend */
	vpos&=0x7ff; if( vpos&0x400 ) vpos |= ~0x7ff; /* sign extend */

	tile_index	= spriteformat16[linkno*4+0];
	format		= spriteformat16[linkno*4+1];
	dx				= spriteformat16[linkno*4+2];
	dy				= spriteformat16[linkno*4+3];
	num_cols		= (format>>4)&0xf;
	num_rows		= (format)&0xf;

	if( num_cols == 0 ) num_cols = 0x10;
	flipx = (hsize&0x8000)?1:0;
	hsize &= 0x3ff;//0x1ff;
	if( hsize == 0 ) return;
	zoomx = (hsize<<16)/(num_cols*16);
	dx = (dx*zoomx+0x8000)>>16;
	if( flipx )
	{
		hpos += dx;
	}
	else
	{
		hpos -= dx;
	}

	if( num_rows == 0 ) num_rows = 0x10;
	flipy = (vsize&0x8000)?1:0;
	vsize &= 0x3ff;
	if( vsize == 0 ) return;
	zoomy = (vsize<<16)/(num_rows*16);
	dy = (dy*zoomy+0x8000)>>16;
	if( flipy )
	{
		vpos += dy;
	}
	else
	{
		vpos -= dy;
	}

	color = (palette&0xf)^mPalXOR;

	source_height_remaining = num_rows*16;
	screen_height_remaining = vsize;
	sy = vpos;
	for( row=0; row<num_rows; row++ )
	{
		tile_screen_height = 16*screen_height_remaining/source_height_remaining;
		zoomy = (screen_height_remaining<<16)/source_height_remaining;
		if( flipy )
		{
			sy -= tile_screen_height;
		}
		source_width_remaining = num_cols*16;
		screen_width_remaining = hsize;
		sx = hpos;
		for( col=0; col<num_cols; col++ )
		{
			tile_screen_width = 16*screen_width_remaining/source_width_remaining;
			zoomx = (screen_width_remaining<<16)/source_width_remaining;
			if( flipx )
			{
				sx -= tile_screen_width;
			}
			tile = spritetile16[tile_index++];
			if( (tile&0x8000)==0 )
			{
				poly3d_DrawTile(
					tile+offset,
					color,
					sx,sy,
					tile_screen_width,
					tile_screen_height,
					flipx, flipy );
			}
			if( !flipx )
			{
				sx += tile_screen_width;
			}
			screen_width_remaining -= tile_screen_width;
			source_width_remaining -= 16;
		} /* next col */
		if( !flipy )
		{
			sy += tile_screen_height;
		}
		screen_height_remaining -= tile_screen_height;
		source_height_remaining -= 16;
	} /* next row */
} /* draw_spriteC355 */

static void
DrawObjectList(
		const rectangle *cliprect,
		int pri,
		const UINT16 *pSpriteList16,
		const UINT16 *pSpriteTable )
{
	UINT16 which;
	int i;
	int count = 0;
	/* count the sprites */
	for( i=0; i<256; i++ )
	{
		which = pSpriteList16[i];
		count++;
		if( which&0x100 ) break;
	}
	/* draw the sprites */
	for( i=0; i<count; i++ )
	{
		which = pSpriteList16[i];
		DrawSpriteC355( cliprect, &pSpriteTable[(which&0xff)*8], pri, i );
	}
} /* DrawObjectList */

void
DrawSprites( const rectangle *cliprect, int pri )
{
	poly3d_Begin2d(mpSpriteTileCache);
//	if( spriteram16[0x18000/2] )
//	{
		DrawObjectList( cliprect,pri,&spriteram16[0x14000/2], &spriteram16[0x10000/2] );
//	}
//	else
//	{
		DrawObjectList( cliprect,pri,&spriteram16[0x02000/2], &spriteram16[0x00000/2] );
//	}
} /* DrawSprites */

extern void namcos21_kickstart( int );

static UINT16 namcos21_video_enable;

READ16_HANDLER(namcos21_video_enable_r)
{
	return namcos21_video_enable;
}

WRITE16_HANDLER(namcos21_video_enable_w)
{
	COMBINE_DATA( &namcos21_video_enable );
}

static UINT16 winrun_color;
static UINT16 winrun_gpu_register[0x10/2];

READ16_HANDLER(winrun_gpu_color_r)
{
	return winrun_color;
}

WRITE16_HANDLER(winrun_gpu_color_w)
{
	COMBINE_DATA( &winrun_color );
}

READ16_HANDLER(winrun_gpu_register_r)
{
	return winrun_gpu_register[offset];
}

WRITE16_HANDLER(winrun_gpu_register_w)
{
	COMBINE_DATA( &winrun_gpu_register[offset] );
}

/*
   0    2    4    6    8    a    c    e
---------------------------------------
0000 fffe 0000 0000 0000 000a 0000 0000 // boot
0001 0000 0002 0000 0000 000a 0000 0000 // title

0001 0000 0002 0000 0000 000a 0004 0000 // attract in-game (0x00)
0001 00ce 0002 0000 0000 000a 00ce 0000 // attract in-game (0x50) ==  80
0001 00ce 0002 0000 0000 000a 00ce 0000 // attract in-game (0x89) == 137

0001 0028 0002 0000 0000 000a 0014 0000 // title
0041 0000 0042 0000 0000 000a 0000 0000 // title(196)
0001 0200 0002 0000 0000 000a 0014 0000 // trans-select
0001 02bc 0002 0000 0000 000a ffff 0000 // trans-select (176)
=======================================
????   dy ???? ---- ---- ---- ---- ----


$e0000c.l
	0x0000 0x007f 0x0080 0x0094 0x00f8 0x00a2 0x00b0 0x00bf 0x00ca 0x00ec 0x00ff

$d00000.l:      $d00004.l
	$180384		 	1+$180384
	0001			 	0x0002
	0101 (347c)	 	0x0102 - manual/auto select
	0041 (3808)	 	0x0042 - title screen (tile, low priority)

$d00002.l
	0x015c
	0x0200
	0x01f0
	0x0268
	0x02bc
	0x035c
	0x0000
	0x00e2+$180386
	0x0200
	0x0028
	0x0050
	0xfffe
	0x015c

$d0000a.l (always 0x000a)
$d0000c.l trigger-only
*/

WRITE16_HANDLER( winrun_gpu_videoram_w)
{
	int color = data>>8;
	int mask  = data&0xff;
	int i;
	for( i=0; i<8; i++ )
	{
		if( mask&(0x01<<i) )
		{
			videoram[(offset+i)&0x7ffff] = color;
		}
	}
} /* winrun_gpu_videoram_w */

READ16_HANDLER( winrun_gpu_videoram_r )
{
	return videoram[offset]<<8;
} /* winrun_gpu_videoram_r */

static int objcode2tile( int code )
{ /* callback for sprite drawing code in namcoic.c */
	return code;
} /* objcode2tile */

VIDEO_START( namcos21 )
{
   mpSpriteTileCache = poly3d_CreateTileCache( 16,16,GetSpriteTileData,NULL,0,0,0 );
	namcos21_video_enable = 0x40;
	if( namcos2_gametype == NAMCOS21_WINRUN91 )
	{
		videoram = auto_malloc(0x80000);
	}
	else
	{
		state_save_register_UINT8( "namcos21", 0/*instance*/, "pointram", namcos21_pointram, 0x20000 );
		namcos21_pointrom = (INT32 *)memory_region( REGION_USER2 );
	}

	state_save_register_UINT16( "namcos21", 0/*instance*/, "SpritePos", mSpritePos, 4 );

	namco_obj_init(
		0,		/* gfx bank */
		0xf,	/* reverse palette mapping */
		objcode2tile );
	return 0;
} /* VIDEO_START( namcos21 ) */

static void
update_palette( void )
{
	int i;
	INT16 data1,data2;
	int r,g,b;
	/*
    Palette:
        0x0000..0x1fff  sprite palettes (0x10 sets of 0x100 colors)

        0x2000..0x3fff  polygon palette bank0
		      (in starblade, some palette animation effects are performed here)
        0x4000..0x5fff  polygon palette bank1
        0x6000..0x7fff  polygon palette bank2
    */    
	for( i=0; i<NAMCOS21_NUM_COLORS; i++ )
	{
		data1 = paletteram16[0x00000/2+i];
		data2 = paletteram16[0x10000/2+i];

		r = data1>>8;
		g = data1&0xff;
		b = data2&0xff;

		palette_set_color( i, r,g,b );
	}
} /* update_palette */

static void
MixBitmapLayer(  mame_bitmap *bitmap, const  rectangle *cliprect )
{
	int bank = 1-mQuadBank;
	int i;
	poly3d_Begin2d(NULL);
	poly3d_NoClip();
	for( i=0; i<mNumQuads[bank]; i++ )
	{
		struct QuadInfo *quad = &mQuadInfo[bank][i];
		if( quad->clip==0 )
		{
			Poly3dVertex v[4];
			int j;
			UINT8 r,g,b;
			palette_get_color(quad->color,&r,&g,&b);
			for( j=0; j<4; j++ )
			{
				float z = quad->zcode[j];
				if( z<0 )
				{
					z = 0;
				}
				else if( z>0x7fff )
				{
					z = 0x7fff;
				}
				v[j].x = quad->sx[j];
				v[j].y = quad->sy[j];
				v[j].z = z;
				v[j].red   = r/255.0f;
				v[j].green = g/255.0f;
				v[j].blue  = b/255.0f;
			}
			poly3d_DrawQuad( NULL, 0, v, 0.0f );
		}
	}
} /* DrawBitmapLayer */

static void
DrawPolyLayer( int bank, int i0, int i1 )
{
	int i;
	poly3d_Begin2d(NULL);
	poly3d_NoClip();
	for( i=i0; i<i1; i++ )
	{
		struct QuadInfo *quad = &mQuadInfo[bank][i];
		if( quad->clip==0 )
		{
			Poly3dVertex v[4];
			int j;
			UINT8 r,g,b;
			palette_get_color(quad->color,&r,&g,&b);
			for( j=0; j<4; j++ )
			{
				float z = quad->zcode[j];
				if( z<0 )
				{
					z = 0;
				}
				else if( z>0x7fff )
				{
					z = 0x7fff;
				}
				v[j].x = quad->sx[j];
				v[j].y = quad->sy[j];
				v[j].z = z;
				v[j].red   = r/255.0f;
				v[j].green = g/255.0f;
				v[j].blue  = b/255.0f;
			}
			poly3d_DrawQuad( NULL, 0, v, 0.0f );
		}
	}
}

static void
MixSpriteLayer(  mame_bitmap *bitmap, const  rectangle *clip )
{ /* blit the visible framebuffer */
	int i;
	int bank = mQuadBank;
	DrawPolyLayer( bank, 0,mSkyQuads[bank] );
	DrawSprites( clip, 0 );
	DrawPolyLayer( bank, mSkyQuads[bank], mNumQuads[bank] );
	DrawSprites( clip, 1 );
} /* MixSpriteLayer */

VIDEO_UPDATE( namcos21 )
{
	{
		UINT8 r,g,b;
		palette_get_color(0xff,&r,&g,&b);
		poly3d_SetBackgroundColor( r,g,b );
	}
	poly3d_Begin();
	if( namcos21_video_enable&0x40 )
	{
		update_palette();
		if( namcos2_gametype == NAMCOS21_WINRUN91 )
		{
			MixBitmapLayer( bitmap, cliprect );
		}
		else
		{
//			if( mbReady )
			{
				namcos21_DrawPolygons();
				mbReady = 0;
			}
			MixSpriteLayer( bitmap, cliprect );
		}
	}
	poly3d_End();
} /* VIDEO_UPDATE( namcos21_default ) */

/*********************************************************************************************/

#define MAX_SURFACE 64
#define MAX_VERTEX	64
extern INT32 ReadPointROMData( unsigned offset );

static void
matrix3d_Multiply( double A[4][4], double B[4][4] )
{
	double temp[4][4];
	int row,col;

	for( row=0;row<4;row++ )
	{
		for(col=0;col<4;col++)
		{
			double sum = 0.0;
			int i;
			for( i=0; i<4; i++ )
			{
				sum += A[row][i]*B[i][col];
			}
			temp[row][col] = sum;
		}
	}
	memcpy( A, temp, sizeof(temp) );
} /* matrix3d_Multiply */

static void
matrix3d_Identity( double M[4][4] )
{
	int r,c;

	for( r=0; r<4; r++ )
	{
		for( c=0; c<4; c++ )
		{
			M[r][c] = (r==c)?1.0:0.0;
		}
	}
} /* matrix3d_Identity */

void
matrix3d_Translate( double M[4][4], double x, double y, double z )
{
	double m[4][4];
	m[0][0] = 1.0;			m[0][1] = 0.0;			m[0][2] = 0.0;			m[0][3] = 0.0;
	m[1][0] = 0.0;			m[1][1] = 1.0;			m[1][2] = 0.0;			m[1][3] = 0.0;
	m[2][0] = 0.0;			m[2][1] = 0.0;			m[2][2] = 1.0;			m[2][3] = 0.0;
	m[3][0] = x;			m[3][1] = y;			m[3][2] = z;			m[3][3] = 1.0;
	matrix3d_Multiply( M, m );
} /* matrix3d_Translate*/

void
matrix3d_Scale( double M[4][4], double x, double y, double z )
{
	double m[4][4];
	m[0][0] = x;			m[0][1] = 0.0;			m[0][2] = 0.0;			m[0][3] = 0.0;
	m[1][0] = 0.0;			m[1][1] = y;			m[1][2] = 0.0;			m[1][3] = 0.0;
	m[2][0] = 0.0;			m[2][1] = 0.0;			m[2][2] = z;			m[2][3] = 0.0;
	m[3][0] = 0.0;			m[3][1] = 0.0;			m[3][2] = 0.0;			m[3][3] = 1.0;
	matrix3d_Multiply( M, m );
} /* matrix3d_Translate*/

static void
matrix3d_RotX( double M[4][4], double thx_sin, double thx_cos )
{
	double m[4][4];
	m[0][0] = 1.0;			m[0][1] = 0.0;			m[0][2] = 0.0;			m[0][3] = 0.0;
	m[1][0] = 0.0;			m[1][1] =  thx_cos;		m[1][2] = thx_sin;		m[1][3] = 0.0;
	m[2][0] = 0.0;			m[2][1] = -thx_sin;		m[2][2] = thx_cos;		m[2][3] = 0.0;
	m[3][0] = 0.0;			m[3][1] = 0.0;			m[3][2] = 0.0;			m[3][3] = 1.0;
	matrix3d_Multiply( M, m );
} /* matrix3d_RotX */

static void
matrix3d_RotY( double M[4][4], double thy_sin, double thy_cos )
{
	double m[4][4];
	m[0][0] = thy_cos;		m[0][1] = 0.0;			m[0][2] = -thy_sin;		m[0][3] = 0.0;
	m[1][0] = 0.0;			m[1][1] = 1.0;			m[1][2] = 0.0;			m[1][3] = 0.0;
	m[2][0] = thy_sin;		m[2][1] = 0.0;			m[2][2] = thy_cos;		m[2][3] = 0.0;
	m[3][0] = 0.0;			m[3][1] = 0.0;			m[3][2] = 0.0;			m[3][3] = 1.0;
	matrix3d_Multiply( M, m );
} /* matrix3d_RotY */

static void
matrix3d_RotZ( double M[4][4], double thz_sin, double thz_cos )
{
	double m[4][4];
	m[0][0] = thz_cos;		m[0][1] = thz_sin;		m[0][2] = 0.0;			m[0][3] = 0.0;
	m[1][0] = -thz_sin;		m[1][1] = thz_cos;		m[1][2] = 0.0;			m[1][3] = 0.0;
	m[2][0] = 0.0;			m[2][1] = 0.0;			m[2][2] = 1.0;			m[2][3] = 0.0;
	m[3][0] = 0.0;			m[3][1] = 0.0;			m[3][2] = 0.0;			m[3][3] = 1.0;
	matrix3d_Multiply( M, m );
} /* matrix3d_RotZ */

struct RotParam
{
	double thx_sin;
	double thx_cos;
	double thy_sin;
	double thy_cos;
	double thz_sin;
	double thz_cos;
	int rolt;
};

static void
namcos3d_Rotate( double M[4][4], const struct RotParam *pParam )
{
	switch( pParam->rolt )
	{
	case 0:
		matrix3d_RotX( M, pParam->thx_sin, pParam->thx_cos );
		matrix3d_RotY( M, pParam->thy_sin, pParam->thy_cos );
		matrix3d_RotZ( M, pParam->thz_sin, pParam->thz_cos );
		break;
	case 1:
		matrix3d_RotX( M, pParam->thx_sin, pParam->thx_cos );
		matrix3d_RotZ( M, pParam->thz_sin, pParam->thz_cos );
		matrix3d_RotY( M, pParam->thy_sin, pParam->thy_cos );
		break;
	case 2:
		matrix3d_RotY( M, pParam->thy_sin, pParam->thy_cos );
		matrix3d_RotX( M, pParam->thx_sin, pParam->thx_cos );
		matrix3d_RotZ( M, pParam->thz_sin, pParam->thz_cos );
		break;
	case 3:
		matrix3d_RotY( M, pParam->thy_sin, pParam->thy_cos );
		matrix3d_RotZ( M, pParam->thz_sin, pParam->thz_cos );
		matrix3d_RotX( M, pParam->thx_sin, pParam->thx_cos );
		break;
	case 4:
		matrix3d_RotZ( M, pParam->thz_sin, pParam->thz_cos );
		matrix3d_RotX( M, pParam->thx_sin, pParam->thx_cos );
		matrix3d_RotY( M, pParam->thy_sin, pParam->thy_cos );
		break;
	case 5:
		matrix3d_RotZ( M, pParam->thz_sin, pParam->thz_cos );
		matrix3d_RotY( M, pParam->thy_sin, pParam->thy_cos );
		matrix3d_RotX( M, pParam->thx_sin, pParam->thx_cos );
		break;
	default:
		printf( "unknown rolt:%08x\n",pParam->rolt );
		break;
	}
} /* namcos3d_Rotate */

#define NEAR_PLANE (10.0f)
//#define NEAR_PLANE ((double)0x112)

static double
interp( double z0, double v0, double z1, double v1 )
{
	double m = (v1-v0)/(z1-z0);
	double b = v0 - m*z0;
	return m*NEAR_PLANE+b;
}

static int mbDspError;

static int
ReadPointROM( int addr )
{
	if( addr<0x400000/4 )
	{
		return namcos21_pointrom[addr];
	}
	else
	{
		printf( "ReadPointROM(0x%x)\n", addr );
		mbDspError = 1;
		return 0xffffff;
	}
}
static int
ReadPointRAM( int addr )
{
	if( addr<0x20000 )
	{
		return namcos21_pointram[addr];
	}
	else
	{
		printf( "ReadPointRAM(0x%x)\n", addr );
		mbDspError = 1;
		return 0xff;
	}
}

static void
BlitPolyObject( int code, double M[4][4] )
{
	INT32 masterAddr;
	double xpos[MAX_VERTEX];
	double ypos[MAX_VERTEX];
	double zpos[MAX_VERTEX];
	int sx[MAX_VERTEX];
	int sy[MAX_VERTEX];
	int zcode[MAX_VERTEX];
	int clip[MAX_VERTEX];

	if( code>=ReadPointROM(0) )
	{
		printf( "BlitPolyObject; bad code=0x%x!\n",code );
		return;
	}
	masterAddr = ReadPointROM(code);
	for(;;)
	{
		INT32 subAddr = ReadPointROM(masterAddr++);
		if( subAddr==0xffffff )
		{
			break;
		}
		else
		{
			int chunkSize    = ReadPointROM(subAddr++);
			int quad_idx     = ReadPointROM(subAddr++);
			int vertexCount  = ReadPointROM(subAddr++)&0xff;
			int zbias        = (INT16)ReadPointROM(subAddr++);
			int count;
			(void)chunkSize;
			if( quad_idx==0xffffff ) return;
			quad_idx*=6;
			if( vertexCount>MAX_VERTEX )
			{
				printf( "vertex overflow\n" );
				mbDspError= 1;
				return;
			}
			for( count=0; count<vertexCount; count++ )
			{
				double x = (INT16)(ReadPointROM(subAddr++)&0xffff);
				double y = (INT16)(ReadPointROM(subAddr++)&0xffff);
				double z = (INT16)(ReadPointROM(subAddr++)&0xffff);
				xpos[count] = M[0][0]*x + M[1][0]*y + M[2][0]*z + M[3][0];
				ypos[count] = M[0][1]*x + M[1][1]*y + M[2][1]*z + M[3][1];
				zpos[count] = M[0][2]*x + M[1][2]*y + M[2][2]*z + M[3][2];
				if( zpos[count]>=NEAR_PLANE )
				{
					sx[count] = NAMCOS21_POLY_FRAME_WIDTH/2 +0x248*xpos[count]/zpos[count];
					sy[count] = NAMCOS21_POLY_FRAME_HEIGHT/2-0x2a0*ypos[count]/zpos[count];
					zcode[count] = zpos[count]+zbias;
					clip[count] = 0;
				}
				else
				{
					clip[count] = 1;
				}
			}
			while( !mbDspError )
			{
				struct QuadInfo *quad = NewQuad();
				int attr  = ReadPointRAM(quad_idx++);
				int color = ReadPointRAM(quad_idx++);
				int j;
				int igood;
				quad->clip = 0;
				quad->color = 0x3e00|color;
				if( (attr&0x02)==0 )
				{
					quad->color|=0x100;
				}
				for( j=0; j<4; j++ )
				{
					int vi = ReadPointRAM(quad_idx+j);
					quad->clip += clip[vi];
					quad->sx[j] = sx[vi];
					quad->sy[j] = sy[vi];
					quad->zcode[j] = zcode[vi];
				}
				switch( quad->clip )
				{
				case 4:
					/* wholly clipped */
					break;
	
				case 3:
					break;

				case 2:
					for( j=0; j<4; j++ )
					{
						int ibad = ReadPointRAM(quad_idx+j);
						if( clip[ibad] )
						{
							igood = ReadPointRAM(quad_idx+((j+1)&3));
							if( clip[igood] )
							{
								igood = ReadPointRAM(quad_idx+((j-1)&3));
							}
							if( !clip[igood] )
							{
								double ix = interp( zpos[ibad],xpos[ibad], zpos[igood], xpos[igood] );
								double iy = interp( zpos[ibad],ypos[ibad], zpos[igood], ypos[igood] );
								quad->sx[j] = NAMCOS21_POLY_FRAME_WIDTH/2 +0x248*ix/NEAR_PLANE;
								quad->sy[j] = NAMCOS21_POLY_FRAME_HEIGHT/2-0x2a0*iy/NEAR_PLANE;
								quad->zcode[j] = NEAR_PLANE+zbias;
								quad->clip--;
							}
						}
					}
					break;

				default:
					break;
				}
				quad_idx+=4;
				if( attr&0x80 )
				{ /* end-of-quadlist marker */
					break;
				}
			} /* next quad */
		} /* subAddr>=0 */
	} /* next subAddr */
} /* BlitPolyObject */

static void
ApplyRotation( const INT16 *pSource, double M[4][4] )
{
	struct RotParam param;
	param.thx_sin = pSource[0]/(double)0x7fff;
	param.thx_cos = pSource[1]/(double)0x7fff;
	param.thy_sin = pSource[2]/(double)0x7fff;
	param.thy_cos = pSource[3]/(double)0x7fff;
	param.thz_sin = pSource[4]/(double)0x7fff;
	param.thz_cos = pSource[5]/(double)0x7fff;
	param.rolt = pSource[6];
	namcos3d_Rotate( M, &param );
} /* ApplyRotation */

static void
ApplyCameraTransformation( const INT16 *pCamera, double M[4][4] )
{
	ApplyRotation( &pCamera[0x40/2], M );
} /* ApplyCameraTransformation */

static int
DrawPolyObject0( const INT16 *pDSPRAM, const INT16 *pCamera )
{
	INT16 code = 1 + pDSPRAM[1];
//	INT16 window = pDSPRAM[2];
	double M[4][4];
	matrix3d_Identity( M );
	matrix3d_Translate( M,pDSPRAM[3],pDSPRAM[4],pDSPRAM[5] );
	ApplyCameraTransformation( pCamera, M );
	BlitPolyObject( code, M );
	return 6;
} /* DrawPolyObject0 */

static int
DrawPolyObject1( const INT16 *pDSPRAM, const INT16 *pCamera )
{
	INT16 code = 1 + pDSPRAM[1];
//	INT16 window = pDSPRAM[2];
	double M[4][4];
	matrix3d_Identity( M );
	ApplyRotation( &pDSPRAM[6], M );
	matrix3d_Translate( M,pDSPRAM[3],pDSPRAM[4],pDSPRAM[5] );
	if( pCamera )
	{
		ApplyCameraTransformation( pCamera, M );
	}
	BlitPolyObject( code, M );
	return 13;
} /* DrawPolyObject1 */

static int
DrawPolyObject4( const INT16 *pDSPRAM, const INT16 *pCamera )
{
	INT16 code = 1 + pDSPRAM[1];
//	INT16 window = pDSPRAM[2];
	double M[4][4];
	matrix3d_Identity( M );
	matrix3d_Scale( M,
		((UINT16)pDSPRAM[13])/(float)0x8000,
		((UINT16)pDSPRAM[14])/(float)0x8000,
		((UINT16)pDSPRAM[15])/(float)0x8000 );
	ApplyRotation( &pDSPRAM[6], M );
	matrix3d_Translate( M,pDSPRAM[3],pDSPRAM[4],pDSPRAM[5] );
	if( pCamera )
	{
		ApplyCameraTransformation( pCamera, M );
	}
	BlitPolyObject( code, M );
	return 13+3;
} /* DrawPolyObject1 */

static void
Transform( double M[4][4], double *px, double *py, double *pz )
{
	double x = *px;
	double y = *py;
	double z = *pz;
	*px = M[0][0]*x + M[1][0]*y + M[2][0]*z + M[3][0];
	*py = M[0][1]*x + M[1][1]*y + M[2][1]*z + M[3][1];
	*pz = M[0][2]*x + M[1][2]*y + M[2][2]*z + M[3][2];
}

static void
DrawGradient( const INT16 *pCamera )
{
	float thx_sin = pCamera[0x40/2+0]/(double)0x7fff;
	float thx_cos = pCamera[0x40/2+1]/(double)0x7fff;
	float thz_sin = pCamera[0x40/2+4]/(double)0x7fff;
	float thz_cos = pCamera[0x40/2+5]/(double)0x7fff;
	int i;
	int count,color,ypos;
	float rx =  thz_cos;
	float ry = -thz_sin;
	float ux = -thz_sin;
	float uy = -thz_cos;
	int cx, cy;
	int i0 = 0;
	int di = 1;
	int zcode = 0x7ff0;//0x7fc0;

	cx = NAMCOS21_POLY_FRAME_WIDTH/2;
	cy = NAMCOS21_POLY_FRAME_HEIGHT/2+thx_sin*0x2a0;
	if( thx_cos<0 )
	{
		rx *= -1;
		ry *= -1;
		ux *= -1;
		uy *= -1;
	}

	/* 0009 0008 ffc0 00ef 00f7 0000 000c */
	ypos  = (UINT16)pCamera[0x52/2];
	count = pCamera[0x50/2];
	color = pCamera[0x56/2];
	for( i=0; i<2; i++ )
	{
		double x0 = cx-rx*0x248*20;
		double y0 = cy-ry*0x2a0*20;
		double x1 = cx+rx*0x248*20;
		double y1 = cy+ry*0x2a0*20;
		while( count-- )
		{
			int ii = i0;
			struct QuadInfo *quad = NewQuad();
			quad->color = 0x2100+color;
			quad->clip = 0;
			quad->zcode[0] = zcode;
			quad->zcode[1] = zcode;
			quad->zcode[2] = zcode;
			quad->zcode[3] = zcode;
			quad->sx[ii] = x0; quad->sy[ii] = y0; ii += di;
			quad->sx[ii] = x1; quad->sy[ii] = y1; ii += di;
			x0 += ux*0x248/146.0f; y0 += uy*0x2a0/146.0f;
			x1 += ux*0x248/146.0f; y1 += uy*0x2a0/146.0f;
			if( count==1 )
			{
				if( i==1 )
				{
					x0 += ux*0x248*20; y0 += uy*0x2a0*20;
					x1 += ux*0x248*20; y1 += uy*0x2a0*20;
				}
				else
				{
					mSkyQuads[mQuadBank]++;
				}
			}
			quad->sx[ii] = x1; quad->sy[ii] = y1; ii += di;
			quad->sx[ii] = x0; quad->sy[ii] = y0; ii += di;
			color++;
		}
		i0 = 3;
		di = -1;
		count = pCamera[0x4e/2];
		color = pCamera[0x54/2];
		zcode = 0x7fc0;
		ux = -ux;
		uy = -uy;
	}
}

void
namcos21_DrawPolygons( void )
{
	int size;
	const INT16 *pCamera;
	const INT16 *pDSPRAM;
	int bank = namcos21_dspram16[0x206/2];

	if( namcos21_dspram16[0x200/2]==0 )
	{
		return; /* hack */
	}

	if( namcos21_dspram16[0x204/2] )
	{
		if( bank==0 || bank==0xffff )
		{ /* wait for 68k ready */
			return;
		}
		pDSPRAM = (INT16 *)&namcos21_dspram16[0x8000/2];
		namcos21_dspram16[0x202/2] = 0; /* clear busy signal */
	}
	else
	{ /* banked display list */
		namcos21_dspram16[0x202/2] = 0; /* clear busy signal */
		if( bank==0 )
		{
			pDSPRAM = (INT16 *)&namcos21_dspram16[0xc000/2];
		}
		else
		{
			pDSPRAM = (INT16 *)&namcos21_dspram16[0x8000/2];
		}
	}

/*
	0000:	0002 0001 2000
	0006:	ffe7 7fff
	000a:	0298 7ff9
	000e:	0000 7fff
	0012:	1000 1000 0000 0000
	001a:	00f8		// WIDTH
	001c:	00f2		// HEIGHT
	001e:	0003
	0020:	0004 008e 0014 0000
	      0000 0000 0000 0000
	0030:	0000 0000 0000 0000
	      0000 0000 0000 0000
	0040:	073c 7fcb	// ROLX
	0044:	0045 7fff	// ROLY
	0048:	edb5 7eae	// ROLZ
	004c:	0002		   // ROLT
	004e: 000f                // #bands 
	0050: 0008 ffc0 00c9 00c1 // #bands,ypos,color0,color1
*/
	pCamera = pDSPRAM;

	pDSPRAM += 0x200/2;
	mbDspError = 0;
	if( mNumQuads[mQuadBank]>mMaxQuads )
	{
		mMaxQuads = mNumQuads[mQuadBank];
		printf( "max polys per frame = %d\n", mMaxQuads );
	}
	mNumQuads[mQuadBank] = 0;
	mSkyQuads[mQuadBank] = 0;
	
	DrawGradient( pCamera );

	for(;;)
	{
		switch( pDSPRAM[0] )
		{
		case 0x0000: /* starblade */
			/* code, win, tx,ty,tz
			 *	[use camera transform]
			 */
			size = DrawPolyObject0( pDSPRAM, pCamera );
			break;

		case 0x0001: /* starblade */
			/* code, win, tx,ty,tz, rolx(2), roly(2), rolz(2), rolt
			 *	[use camera transform]
			 */
			size = DrawPolyObject1( pDSPRAM, pCamera );
			break;

		case 0x0002: /* starblade */
			/* code, win, tx,ty,tz, rolx(2), roly(2), rolz(2), rolt
			 *	[local transform only]
			 */
			size = DrawPolyObject1( pDSPRAM, NULL );
			break;

		case 0x0004: /* air combat (& solvalou) */
			/* code, win, tx,ty,tz, rolx(2), roly(2), rolz(2), rolt, scalex, scaley, scalez
			 *	[use camera transform]
			 */
			size = DrawPolyObject4( pDSPRAM, pCamera );
			break;
#if 0
		case 0x0005: /* air combat */
			size = DrawPolyObject0( pDSPRAM, pCamera );
			break;

		case 0x0006: /* air combat */
			size = DrawPolyObject1( pDSPRAM, pCamera );
			break;

		case 0x0007: /* air combat */
			size = DrawPolyObject1( pDSPRAM, pCamera );
			/* 0x00af 0x0003
			 * 0x3518 0xe889 0xe39c
			 * 0x0000 0x7fff 0x70e0 0xc3a7 0x0000 0x7fff 0x0004
			 */
			break;
#endif
		case 0x100: /* special end-marker for CyberSled */
		case (INT16)0xffff: /* end-of-list marker */
			return;

		default:
			printf( "***unknown obj type!\n%04x %04x %04x %04x %04x %04x %04x %04x\n",
				pDSPRAM[0],
				pDSPRAM[1],
				pDSPRAM[2],
				pDSPRAM[3],
				pDSPRAM[4],
				pDSPRAM[5],
				pDSPRAM[6],
				pDSPRAM[7] );

			if(0)
			{
				char path[245];
				FILE *f;
				static int count;
				sprintf( path, "dump%d.bin", count++ );
				f = fopen( path, "wb" );
				if( f )
				{
					fwrite( namcos21_dspram16,0x10000, 1, f );
					fclose( f );
				}
			}
			return;
		}
		if( mbDspError ) return;
		pDSPRAM += size;
	} /* next object */
} /* namcos21_DrawPolygons */

static void
SimulateDSPs( void )
{
	/* some DSP witchery follows; it's an attempt to simulate the DSP behavior that
	 * Starblade expects during setup.
	 */
	const UINT16 cmd1[] =
	{
		0x0000,0x0001,0x0002,0x000a,0xcca3,0x0000,0x0000,0x0000,
		0x0000,0x0002,0x0000,0x0000,0x0001,0x0000,0x0000,0x0000,
		0x0080,0x0004,0xffff,0xffff
	};
	if( memcmp( &namcos21_dspram16[0x100/2], cmd1, sizeof(cmd1) )==0 )
	{ /* the check above is done so we don't interfere with the DSPRAM test */
		namcos21_dspram16[0x112/2] = 0; /* status to fake working DSP */
		namcos21_dspram16[0x100/2] = 2; /* status to fake working DSP */
		namcos21_dspram16[0x102/2] = 2; /* status to fake working DSP */
		namcos21_dspram16[0x110/2] = 2; /* status to fake working DSP */
		namcos21_dspram16[0x10c/2] = 0xd5df; /* checksum computed by DSP */
		namcos21_dspram16[0x10a/2] = 0xed53; /* checksum computed by DSP */
	}
	else if( namcos21_dspram16[0x10e/2] == 0x0001 )
	{
		/* This signals that a large chunk of code/data has been written by the main CPU.
		 *
		 * Presumably the DSP processor(s) copy it to private RAM at this point.
		 * The main CPU waits for this flag to be cleared.
		 */
		namcos21_dspram16[0x10e/2] = 0; /* ack */
	}
}

void
namcos21_kickstart( int data )
{
	if( namcos2_gametype==NAMCOS21_WINRUN91 ||
		 namcos2_gametype==NAMCOS21_DRIVERS_EYES )
	{
		if( !winrun_irq_enable )
		{
			return;
		}
		cpunum_set_input_line(4, 0, HOLD_LINE);
		return;
	}
//	printf( "C148:0x%x\n",data);
	if(data&4) mbReady = 1;//namcos21_DrawPolygons();
#if 0
	{ /* patch watchdog */
		UINT16 *pMem = (UINT16 *)memory_region(CPU_DSP_MASTER_REGION);
		switch( namcos2_gametype )
		{
		case NAMCOS21_AIRCOMBAT:
			pMem[0x808e] = 0x808f;
			break;
		case NAMCOS21_SOLVALOU:
			pMem[0x808b] = 0x808c;
			break;
		}
	}

	logerror( "kickstart\n" );
	namcos21_ClearPolyFrameBuffer();
	mpDspState->masterSourceAddr = 0;
	mpDspState->slaveOutputSize = 0;
	mpDspState->masterFinished = 0;
	mpDspState->slaveActive = 0;
	cpunum_set_input_line(4, 0, HOLD_LINE); /* DSP: master */
	cpunum_set_input_line(5, INPUT_LINE_RESET, PULSE_LINE); /* DSP: slave */
#endif
}

