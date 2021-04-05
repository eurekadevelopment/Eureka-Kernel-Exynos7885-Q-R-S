/****************************************************************************
 *
 * Copyright (c) 2014 - 2016 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#define PCI_DEVICE_ID_SAMSUNG_SCSC 0x7011
#define DRV_NAME "scscPCIe"

/* Max amount of memory allocated by dma_alloc_coherent */
#define PCIE_MIF_PREALLOC_MEM   (4 * 1024 * 1024)
/* Allocatable memory for upper layers */
/* This value should take into account PCIE_MIF_PREALLOC_MEM - mbox/register
 * emulation - peterson mutex ex: */
/* -------------------- PCIE_MIF_PREALLOC_MEM
 |     scsc_mbox_s  |
 | --------------------
 |     peterson_m   |
 | --------------------
 |  /////////////// |
 | -------------------  PCIE_MIF_ALLOC_MEM
 |  alloc memory    |
 |                  |
 |                  |
 | --------------------
 */
#define PCIE_MIF_ALLOC_MEM      (3 * 1024 * 1024)
