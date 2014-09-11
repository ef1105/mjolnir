/**
 * @file win_poly3d.h
 *
 * defines prototypes for win32-specific init/term routines.
 */

#ifndef WIN_POLY3D_H
#define WIN_POLY3D_H

#include "poly3d.h"
#include <Windows.h>

/* win32 specific glue */
int poly3d_Init( HWND hWnd, int width, int height );
void poly3d_Term( HWND hWnd );
void poly3d_ResizeWindow( int width, int height );

#endif /* WIN_POLY3D_H */
