#------------------------------------------------------------------------------
# HALCON/C++ examples
#
# General macro definitions for HALCON/C++ examples
#
#------------------------------------------------------------------------------
#  Copyright (c) 1996-2015
#  MVTec Software GmbH, Muenchen, Germany
#  www.mvtec.com
#------------------------------------------------------------------------------

#BASE_BIN_DIR   = $(BASE_DIR)bin$(CONF_SUFFIX)$(SL)
#BIN_DIR        = $(BASE_BIN_DIR)$(HALCONARCH)$(SL)
BASE_BIN_DIR   = $(BASE_DIR)
BIN_DIR        = $(BASE_BIN_DIR)
HINC_DIR       = $(HALCONROOT)$(SL)include
HCPPINC_DIR    = $(HALCONROOT)$(SL)include$(SL)halconcpp
HLIB_DIR       = $(HALCONROOT)$(SL)lib$(SL)$(HALCONARCH)$(SL)
