
#ifndef VTKIOEXPORTOPENGL_EXPORT_H
#define VTKIOEXPORTOPENGL_EXPORT_H

#ifdef VTKIOEXPORTOPENGL_STATIC_DEFINE
#  define VTKIOEXPORTOPENGL_EXPORT
#  define VTKIOEXPORTOPENGL_NO_EXPORT
#else
#  ifndef VTKIOEXPORTOPENGL_EXPORT
#    ifdef vtkIOExportOpenGL_EXPORTS
        /* We are building this library */
#      define VTKIOEXPORTOPENGL_EXPORT 
#    else
        /* We are using this library */
#      define VTKIOEXPORTOPENGL_EXPORT 
#    endif
#  endif

#  ifndef VTKIOEXPORTOPENGL_NO_EXPORT
#    define VTKIOEXPORTOPENGL_NO_EXPORT 
#  endif
#endif

#ifndef VTKIOEXPORTOPENGL_DEPRECATED
#  define VTKIOEXPORTOPENGL_DEPRECATED __declspec(deprecated)
#  define VTKIOEXPORTOPENGL_DEPRECATED_EXPORT VTKIOEXPORTOPENGL_EXPORT __declspec(deprecated)
#  define VTKIOEXPORTOPENGL_DEPRECATED_NO_EXPORT VTKIOEXPORTOPENGL_NO_EXPORT __declspec(deprecated)
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKIOEXPORTOPENGL_NO_DEPRECATED
#endif

/* AutoInit dependencies.  */
#include "vtkIOExportModule.h"
#include "vtkIOExportModule.h"
#include "vtkRenderingOpenGLModule.h"

/* AutoInit implementations.  */
#if defined(vtkIOExportOpenGL_INCLUDE)
# include vtkIOExportOpenGL_INCLUDE
#endif
#if defined(vtkIOExportOpenGL_AUTOINIT)
# include "vtkAutoInit.h"
VTK_AUTOINIT(vtkIOExportOpenGL)
#endif

#endif
