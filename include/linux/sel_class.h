//////////////////////////////////////////////////////////////////////////////
// COPYRIGHT (c) 2009 Schweitzer Engineering Laboratories, Inc.
///
/// @file sel_class.h
///
/// @brief devfs support for sel devices.
//////////////////////////////////////////////////////////////////////////////

extern void sel_register( dev_t devt, char const *name, ... )
	                                  __attribute__((format(printf,2,3)));
extern void sel_deregister( dev_t devt );
