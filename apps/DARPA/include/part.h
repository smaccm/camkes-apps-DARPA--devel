/*
 * Copyright (c) 2014 National ICT Australia Limited (NICTA), ABN 62 102 206 173.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _PART_H_
#define _PART_H_

//common file
typedef unsigned long ulong;
typedef unsigned char uchar;


typedef struct block_dev_desc {
	int		if_type;	/* type of the interface */
	int		dev;		/* device number */
	unsigned char	part_type;	/* partition type */
	unsigned char	target;		/* target SCSI ID */
	unsigned char	lun;		/* target LUN */
	unsigned char	type;		/* device type */
	unsigned char	removable;	/* removable device */
#ifdef CONFIG_LBA48
	unsigned char	lba48;		/* device can use 48bit addr (ATA/ATAPI v7) */
#endif
	ulong			lba;		/* number of blocks */
	unsigned long	blksz;		/* block size */
	int		log2blksz;	/* for convenience: log2(blksz) */
	char		vendor [40+1];	/* IDE model, SCSI Vendor */
	char		product[20+1];	/* IDE Serial no, SCSI product */
	char		revision[8+1];	/* firmware revision */
	unsigned long	(*block_read)(int dev,
				      ulong start,
				      ulong blkcnt,
				      void *buffer);
	unsigned long	(*block_write)(int dev,
				       ulong start,
				       ulong blkcnt,
				       const void *buffer);
	unsigned long   (*block_erase)(int dev,
				       ulong start,
				       ulong blkcnt);
	void		*priv;		/* driver private struct pointer */
}block_dev_desc_t;


#define BLOCK_CNT(size, block_dev_desc) (PAD_COUNT(size, block_dev_desc->blksz))
#define PAD_TO_BLOCKSIZE(size, block_dev_desc) \
	(PAD_SIZE(size, block_dev_desc->blksz))
#define LOG2(x) (((x & 0xaaaaaaaa) ? 1 : 0) + ((x & 0xcccccccc) ? 2 : 0) + \
		 ((x & 0xf0f0f0f0) ? 4 : 0) + ((x & 0xff00ff00) ? 8 : 0) + \
		 ((x & 0xffff0000) ? 16 : 0))
#define LOG2_INVALID(type) ((type)((sizeof(type)<<3)-1))

/* Interface types: */
#define IF_TYPE_UNKNOWN		0
#define IF_TYPE_IDE		1
#define IF_TYPE_SCSI		2
#define IF_TYPE_ATAPI		3
#define IF_TYPE_USB		4
#define IF_TYPE_DOC		5
#define IF_TYPE_MMC		6
#define IF_TYPE_SD		7
#define IF_TYPE_SATA		8

/* Part types */
#define PART_TYPE_UNKNOWN	0x00
#define PART_TYPE_MAC		0x01
#define PART_TYPE_DOS		0x02
#define PART_TYPE_ISO		0x03
#define PART_TYPE_AMIGA		0x04
#define PART_TYPE_EFI		0x05

/*
 * Type string for U-Boot bootable partitions
 */
#define BOOT_PART_TYPE	"U-Boot"	/* primary boot partition type	*/
#define BOOT_PART_COMP	"PPCBoot"	/* PPCBoot compatibility type	*/

/* device types */
#define DEV_TYPE_UNKNOWN	0xff	/* not connected */
#define DEV_TYPE_HARDDISK	0x00	/* harddisk */
#define DEV_TYPE_TAPE		0x01	/* Tape */
#define DEV_TYPE_CDROM		0x05	/* CD-ROM */
#define DEV_TYPE_OPDISK		0x07	/* optical disk */

typedef struct disk_partition {
	ulong	start;		/* # of first block in partition	*/
	ulong	size;		/* number of blocks in partition	*/
	ulong	blksz;		/* block size in bytes			*/
	uchar	name[32];	/* partition name			*/
	uchar	type[32];	/* string type description		*/
	int	bootable;	/* Active/Bootable flag is set		*/
#ifdef CONFIG_PARTITION_UUIDS
	char	uuid[37];	/* filesystem UUID as string, if exists	*/
#endif
} disk_partition_t;


#endif
