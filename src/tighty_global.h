#ifndef TIGHTY_GLOBAL_H
#define TIGHTY_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(TIGHTY_LIBRARY)
#  define TIGHTYSHARED_EXPORT Q_DECL_EXPORT
#else
#  define TIGHTYSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // TIGHTY_GLOBAL_H
