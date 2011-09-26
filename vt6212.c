/**********************************************************************
 *  PymbleSoftware Pty Ltd, BSDero and nekoware contriubtors. 
**********************************************************************/

#include <sys/types.h>
#include <sys/cpu.h>
#include <sys/systm.h>
#include <sys/cmn_err.h>
#include <sys/errno.h>
#include <sys/buf.h>
#include <sys/ioctl.h>
#include <sys/cred.h>
#include <ksys/ddmap.h>
#include <sys/poll.h>
#include <sys/invent.h>
#include <sys/debug.h>
#include <sys/sbd.h>
#include <sys/kmem.h>
#include <sys/edt.h>
#include <sys/dmamap.h>
#include <sys/hwgraph.h>
#include <sys/iobus.h>
#include <sys/iograph.h>
#include <sys/param.h>
#include <sys/pio.h>
#include <sys/sema.h>
#include <sys/ddi.h>
#include <sys/atomic_ops.h>
#include <sys/PCI/PCI_defs.h>
#include <sys/PCI/pciio.h>
#define NEW(ptr)         (ptr = kmem_alloc(sizeof (*(ptr)), KM_SLEEP))
#define DEL(ptr)         (kmem_free(ptr, sizeof (*(ptr))))
/*
 *    vt6212: a generic device driver for a generic PCI device.
 */
int         vt6212_devflag = D_MP;
int         vt6212_inuse = 0;      /* number of "vt6212" devices open */
/* ====================================================================
 *          Device-Related Constants and Structures
 */
#define VT6212_VENDOR_ID_NUM      0x1106
#define VT6212_DEVICE_ID_NUM      0x3808
/*
 *    All registers on the Sample PCIIO Client
 *      device are 32 bits wide.
 */
typedef __uint32_t       vt6212_reg_t;
typedef volatile struct vt6212_regs_s *vt6212_regs_t; /* dev registers */
typedef struct vt6212_soft_s *vt6212_soft_t;          /* software state */
/*
 *    vt6212_regs: layout of device registers
 *      Our device config registers are, of course, at
 *      the base of our assigned CFG space.
 *      Our sample device registers are in the PCI area
 *      decoded by the device's first BASE_ADDR window.
 */
struct vt6212_regs_s {
    vt6212_reg_t              pr_control;
    vt6212_reg_t              pr_status;
};
struct vt6212_soft_s {
    vertex_hdl_t       vt_conn;    /* connection for pci services */
    vertex_hdl_t       vt_vhdl;    /* backpointer to device vertex */
    vertex_hdl_t       vt_blockv; /* backpointer to block vertex */
    vertex_hdl_t       vt_charv;   /* backpointer to char vertex */
    volatile uchar_t vt_cfg;       /* cached ptr to my config regs */
    vt6212_regs_t       vt_regs;    /* cached ptr to my regs */
    pciio_piomap_t     vt_cmap;    /* piomap (if any) for vt_cfg */
    pciio_piomap_t     vt_rmap;    /* piomap (if any) for vt_regs */
    unsigned           vt_sst;     /* driver "software state" */
#define VT6212_SST_RX_READY       (0x0001)
#define VT6212_SST_TX_READY       (0x0002)
#define VT6212_SST_ERROR          (0x0004)
#define VT6212_SST_INUSE          (0x8000)
    pciio_intr_t       vt_intr;       /* pciio intr for INTA and INTB */
    pciio_dmamap_t     vt_ctl_dmamap; /* control channel dma mapping */
    pciio_dmamap_t     vt_str_dmamap; /* stream channel dma mapping */
    struct pollhead    *vt_pollhead; /* for poll() */
    int                vt_blocks; /* block dev size in NBPSCTR blocks*/
};
#define vt6212_soft_set(v,i)      device_info_set((v),(void *)(i))
#define vt6212_soft_get(v)        ((vt6212_soft_t)device_info_get((v)))
/*=====================================================================
 *           FUNCTION TABLE OF CONTENTS
 */
void                   vt6212_init(void);
int                    vt6212_unload(void);

int                    vt6212_unreg(void);
int                    vt6212_attach(vertex_hdl_t conn);
int                    vt6212_detach(vertex_hdl_t conn);
static pciio_iter_f    vt6212_reloadme;
static pciio_iter_f    vt6212_unloadme;
int                    vt6212_open(dev_t *devp, int oflag, int otyp,
                                  cred_t *crp);
int                    vt6212_close(dev_t dev, int oflag, int otyp,
                                   cred_t *crp);
int                    vt6212_ioctl(dev_t dev, int cmd, void *arg,
                                   int mode, cred_t *crp, int *rvalp);
int                    vt6212_read(dev_t dev, uio_t * uiop, cred_t *crp);
int                    vt6212_write(dev_t dev, uio_t * uiop,cred_t *crp);
int                    vt6212_strategy(struct buf *bp);
int                    vt6212_poll(dev_t dev, short events, int anyyet,
                                short *reventsp, struct pollhead **phpp,
                                unsigned int *genp);
int                    vt6212_map(dev_t dev, vhandl_t *vt,
                                 off_t off, size_t len, uint_t prot);
int                    vt6212_unmap(dev_t dev, vhandl_t *vt);
void                   vt6212_dma_intr(intr_arg_t arg);
static error_handler_f vt6212_error_handler;
void                   vt6212_halt(void);
int                    vt6212_size(dev_t dev);
int                    vt6212_print(dev_t dev, char *str);
/*=====================================================================
  *                  Driver Initialization
  */
/*
  *    vt6212_init: called once during system startup or
  *      when a loadable driver is loaded.
  */
void
vt6212_init(void)
{
     printf("vt6212_init()\n");
     /*
      * if we are already registered, note that this is a
      * "reload" and reconnect all the places we attached.
      */
     pciio_iterate("vt6212_", vt6212_reloadme);

}

/*
  *    vt6212_unload: if no "vt6212" is open, put us to bed
  *      and let the driver text get unloaded.
  */
int
vt6212_unload(void)
{
     if (vt6212_inuse)
         return EBUSY;
     pciio_iterate("vt6212_", vt6212_unloadme);
     return 0;
}
/*
  *    vt6212_reg: called once during system startup or
  *      when a loadable driver is loaded.
  *    NOTE: a bus provider register routine should always be
  *      called from _reg, rather than from _init. In the case
  *      of a loadable module, the devsw is not hooked up
  *      when the _init routines are called.
  */
int
vt6212_reg(void)
{
     printf("vt6212_reg()\n");
     pciio_driver_register(VT6212_VENDOR_ID_NUM,
                           VT6212_DEVICE_ID_NUM,
                           "vt6212_",
                           0);
     return 0;
}
/*
  *    vt6212_unreg: called when a loadable driver is unloaded.
  */
int
vt6212_unreg(void)
{
     pciio_driver_unregister("vt6212_");
     return 0;
}
/*
  *    vt6212_attach: called by the pciio infrastructure
  *      once for each vertex representing a crosstalk widget.
  *      In large configurations, it is possible for
  *      huge number of CPUs to enter this routine all at
  *      nearly the same time, for different specific
  *      instances of the device. Attempting to give your
  *      devices sequence numbers based on the order they
  *      are found in the system is not only futile but may be
  *      dangerous as the order may differ from run to run.
  */
int
vt6212_attach(vertex_hdl_t conn)
{
     vertex_hdl_t             vhdl, blockv, charv;
     volatile uchar_t        *cfg;
     vt6212_regs_t             regs;
     vt6212_soft_t             soft;
     pciio_piomap_t           cmap = 0;
     pciio_piomap_t           rmap = 0;
     printf("vt6212_attach()\n");
     hwgraph_device_add(conn,"vt6212","vt6212_",&vhdl,&blockv,&charv);
     /*
      * Allocate a place to put per-device information for this vertex.
      * Then associate it with the vertex in the most efficient manner.
      */
     NEW(soft);
     ASSERT(soft != NULL);
     vt6212_soft_set(vhdl, soft);
     vt6212_soft_set(blockv, soft);
     vt6212_soft_set(charv, soft);
     soft->vt_conn = conn;
     soft->vt_vhdl = vhdl;
     soft->vt_blockv = blockv;
     soft->vt_charv = charv;
     /*
      * Find our PCI CONFIG registers.
      */
     cfg = (volatile uchar_t *) pciio_pio_addr
         (conn, 0,                /* device and (override) dev_info */
          PCIIO_SPACE_CFG,        /* select configuration addr space */
          0,                      /* from the start of space, */
          PCI_CFG_VEND_SPECIFIC, /* ... up to vendor specific stuff */
          &cmap,                  /* in case we needed a piomap */
          0);                     /* flag word */
/***     soft->vt_cfg = cfg;   ***/       /* save for later */
     soft->vt_cmap = cmap;
/****      printf("vt6212_attach: I can see my CFG regs at 0x%x\n", ***/
    /*
     * Get a pointer to our DEVICE registers
     */

    regs = (vt6212_regs_t) pciio_pio_addr
        ( conn, 0,               /* device and (override) dev_info */
         PCIIO_SPACE_WIN(0),    /* in my primary decode window, */
         0, sizeof(*regs),      /* base and size */
         &rmap,                 /* in case we needed a piomap */
         0);                    /* flag word */

    soft->vt_regs = regs;       /* save for later */
    soft->vt_rmap = rmap;
    printf("vt6212_attach: I can see my device regs at 0x%x\n", regs);
    /*
     * Set up our interrupt.
     * We might interrupt on INTA or INTB,
     * but route 'em both to the same function.
     */
    soft->vt_intr = pciio_intr_alloc
        (conn, 0,
         PCIIO_INTR_LINE_A |
         PCIIO_INTR_LINE_B,
         vhdl);
    pciio_intr_connect(soft->vt_intr,
                       vt6212_dma_intr, soft,(void *) 0);
    /*
     * set up our error handler.
     */
    pciio_error_register(conn, vt6212_error_handler, soft);
    /*
     * For pciio clients, *now* is the time to
     * allocate pollhead structures.
     */
    soft->vt_pollhead = phalloc(0);
    return 0;                       /* attach successsful */
}
/*
  *   vt6212_detach: called by the pciio infrastructure
  *     once for each vertex representing a crosstalk
  *     widget when unregistering the driver.
  *
  *     In large configurations, it is possible for a
  *     huge number of CPUs to enter this routine all at
  *     nearly the same time, for different specific
  *     instances of the device. Attempting to give
  *       devices sequence numbers based on the order they
  *       are found in the system is not only futile but may be
  *       dangerous as the order may differ from run to run.
  */
int
vt6212_detach(vertex_hdl_t conn)
{
     vertex_hdl_t             vhdl, blockv, charv;
     vt6212_soft_t             soft;
     printf("vt6212_detach()\n");
     if (GRAPH_SUCCESS !=
          hwgraph_traverse(conn, "vt6212", &vhdl))
          return -1;
     soft = vt6212_soft_get(vhdl);
     pciio_error_register(conn, 0, 0);
     pciio_intr_disconnect(soft->vt_intr);
     pciio_intr_free(soft->vt_intr);
     phfree(soft->vt_pollhead);
     if (soft->vt_ctl_dmamap)
          pciio_dmamap_free(soft->vt_ctl_dmamap);
     if (soft->vt_str_dmamap)
          pciio_dmamap_free(soft->vt_str_dmamap);
     if (soft->vt_cmap)
          pciio_piomap_free(soft->vt_cmap);
     if (soft->vt_rmap)
          pciio_piomap_free(soft->vt_rmap);
     hwgraph_edge_remove(conn, "vt6212", &vhdl);
     /*
       * we really need "hwgraph_dev_remove" ...
       */
     if (GRAPH_SUCCESS ==
          hwgraph_edge_remove(vhdl, EDGE_LBL_BLOCK, &blockv)) {
          vt6212_soft_set(blockv, 0);
          hwgraph_vertex_destroy(blockv);
     }
     if (GRAPH_SUCCESS ==
          hwgraph_edge_remove(vhdl, EDGE_LBL_CHAR, &charv)) {
          vt6212_soft_set(charv, 0);
          hwgraph_vertex_destroy(charv);
     }
     vt6212_soft_set(vhdl, 0);
     hwgraph_vertex_destroy(vhdl);
     DEL(soft);
     return
0;
}
/*
  *    vt6212_reloadme: utility function used indirectly
  *      by vt6212_init, via pciio_iterate, to "reconnect"
  *      each connection point when the driver has been
  *      reloaded.
  */
static void
vt6212_reloadme(vertex_hdl_t conn)
{
     vertex_hdl_t            vhdl;
     vt6212_soft_t            soft;
     if (GRAPH_SUCCESS !=
         hwgraph_traverse(conn, "vt6212", &vhdl))
         return;
     soft = vt6212_soft_get(vhdl);
     /*
      * Reconnect our error and interrupt handlers
      */
     pciio_error_register(conn, vt6212_error_handler, soft);
     pciio_intr_connect(soft->vt_intr, vt6212_dma_intr, soft, 0);
}
/*
  *    vt6212_unloadme: utility function used indirectly by
  *      vt6212_unload, via pciio_iterate, to "disconnect" each
  *      connection point before the driver becomes unloaded.
  */
static void
vt6212_unloadme(vertex_hdl_t pconn)
{
     vertex_hdl_t            vhdl;
     vt6212_soft_t            soft;
     if (GRAPH_SUCCESS !=
         hwgraph_traverse(pconn, "vt6212", &vhdl))
         return;
     soft = vt6212_soft_get(vhdl);
     /*
      * Disconnect our error and interrupt handlers
      */
     pciio_error_register(pconn, 0, 0);
     pciio_intr_disconnect(soft->vt_intr);
}

/* ====================================================================
  *          DRIVER OPEN/CLOSE
  */
/*
  *    vt6212_open: called when a device special file is
  *      opened or when a block device is mounted.
  */
/* ARGSUSED */
int
vt6212_open(dev_t *devp, int oflag, int otyp, cred_t *crp)
{
     vertex_hdl_t            vhdl = dev_to_vhdl(*devp);
     vt6212_soft_t            soft = vt6212_soft_get(vhdl);
     vt6212_regs_t            regs = soft->vt_regs;
     printf("vt6212_open() regs=%x\n", regs);
     /*
      * BLOCK DEVICES: now would be a good time to
      * calculate the size of the device and stash it
      * away for use by vt6212_size.
      */
     /*
      * USER ABI (64-bit): chances are, you are being
      * compiled for use in a 64-bit IRIX kernel; if
      * you use the _ioctl or _poll entry points, now
      * would be a good time to test and save the
      * user process' model so you know how to
      * interpret the user ioctl and poll requests.
      */
     if (!(VT6212_SST_INUSE & atomicSetUint(&soft->vt_sst,
VT6212_SST_INUSE)))
         atomicAddInt(&vt6212_inuse, 1);
     return 0;
}
/*
  *    vt6212_close: called when a device special file
  *      is closed by a process and no other processes
  *      still have it open ("last close").
  */
/* ARGSUSED */
int
vt6212_close(dev_t dev, int oflag, int otyp, cred_t *crp)
{
     vertex_hdl_t            vhdl =
dev_to_vhdl(dev);
     vt6212_soft_t            soft = vt6212_soft_get(vhdl);
     vt6212_regs_t            regs = soft->vt_regs;
     printf("vt6212_close() regs=%x\n", regs);
     atomicClearUint(&soft->vt_sst, VT6212_SST_INUSE);
     atomicAddInt(&vt6212_inuse, -1);
     return 0;
}
/* ====================================================================
  *          CONTROL ENTRY POINT
  */
/*
  *    vt6212_ioctl: a user has made an ioctl request
  *      for an open character device.
  *      Arguments cmd and arg are as specified by the user;
  *      arg is probably a pointer to something in the user's
  *      address space, so you need to use copyin() to
  *      read through it and copyout() to write through it.
  */
/* ARGSUSED */
int
vt6212_ioctl(dev_t dev, int cmd, void *arg,
             int mode, cred_t *crp, int *rvalp)
{
     vertex_hdl_t            vhdl = dev_to_vhdl(dev);
     vt6212_soft_t            soft = vt6212_soft_get(vhdl);
     vt6212_regs_t            regs = soft->vt_regs;
     printf("vt6212_ioctl() regs=%x\n", regs);
     *rvalp = -1;
     return ENOTTY;          /* TeleType® is a registered trademark */
}
/* ====================================================================
  *          DATA TRANSFER ENTRY POINTS
  *      Since I'm trying to provide an example for both
  *      character and block devices, I'm routing read
  *      and write back through strategy as described in
  *      the IRIX Device Driver Programming Guide.
  *      This limits our character driver to reading and
  *      writing in multiples of the standard sector length.
  */
/* ARGSUSED */
int
vt6212_read(dev_t dev, uio_t * uiop, cred_t
* crp )
{
     return physiock(vt6212_strategy,
                     0,          /* alocate temp buffer & buf_t */
                     dev,        /* dev_t arg for strategy */
                     B_READ,     /* direction flag for buf_t */
                     vt6212_size(dev),
                     uiop);
}
/* ARGSUSED */
int
vt6212_write(dev_t dev, uio_t * uiop, cred_t *crp)
{
     return physiock(vt6212_strategy,
                     0,          /* alocate temp buffer & buf_t */
                     dev,        /* dev_t arg for strategy */
                     B_WRITE,    /* direction flag for buf_t */
                     vt6212_size(dev),
                     uiop);
}
/* ARGSUSED */
int
vt6212_strategy(struct buf *bp)
{
     /*
      * XXX - create strategy code here.
      */
     return 0;
}
/* ====================================================================
  *          POLL ENTRY POINT
  */
int
vt6212_poll(dev_t dev, short events, int anyyet,
            short *reventsp, struct pollhead **phpp, unsigned int *genp)
{
     vertex_hdl_t            vhdl = dev_to_vhdl(dev);
     vt6212_soft_t            soft = vt6212_soft_get(vhdl);
     vt6212_regs_t            regs = soft->vt_regs;
     short                   happened = 0;
     unsigned int            gen;
     printf("vt6212_poll() regs=%x\n", regs);
     /*
      * Need to snapshot the pollhead generation number before we check
      * device state. In many drivers a lock is used to interlock
the
       * "high" and "low" portions of the driver. In those cases we can
       * wait to do this snapshot till we're in the critical region.
       * Snapshotting it early isn't a problem since that makes the
       * snapshotted generation number a more conservative estimate of
       * what generation of pollhead our event state report indicates.
       */
     gen = POLLGEN(soft->vt_pollhead);
     if (events & (POLLIN | POLLRDNORM))
          if (soft->vt_sst & VT6212_SST_RX_READY)
              happened |= POLLIN | POLLRDNORM;
     if (events & POLLOUT)
          if (soft->vt_sst & VT6212_SST_TX_READY)
              happened |= POLLOUT;
     if (soft->vt_sst & VT6212_SST_ERROR)
          happened |= POLLERR;
     *reventsp = happened;
     if (!happened && anyyet) {
          *phpp = soft->vt_pollhead;
          *genp = gen;
     }
     return 0;
}
/* ====================================================================
  *           MEMORY MAP ENTRY POINTS
  */
/* ARGSUSED */
int
vt6212_map(dev_t dev, vhandl_t *vt,
            off_t off, size_t len, uint_t prot)
{
     vertex_hdl_t             vhdl = dev_to_vhdl(dev);
     vt6212_soft_t             soft = vt6212_soft_get(vhdl);
     vertex_hdl_t             conn = soft->vt_conn;
     vt6212_regs_t             regs = soft->vt_regs;
     pciio_piomap_t           amap = 0;
     caddr_t                  kaddr;
     printf("vt6212_map() regs=%x\n", regs);
     /*
       * Stuff we want users to mmap is in our second BASE_ADDR window.
       */
     kaddr = (caddr_t) pciio_pio_addr
          (conn, 0,
	PCIIO_SPACE_WIN(1),
          off, len, &amap, 0);
     if (kaddr == NULL)
         return EINVAL;
     /*
      * XXX - must stash amap somewhere so we can pciio_piomap_free it
      * when the mapping goes away.
      */
     v_mapphys(vt, kaddr, len);
     return 0;
}
/* ARGSUSED2 */
int
vt6212_unmap(dev_t dev, vhandl_t *vt)
{
     /*
      * XXX - need to find "amap" that we used in vt6212_map() above,
      * and if (amap) pciio_piomap_free(amap);
      */
     return (0);
}
/* ====================================================================
  *          INTERRUPT ENTRY POINTS
  * We avoid using the standard name, since our prototype has changed.
  */
void
vt6212_dma_intr(intr_arg_t arg)
{
     vt6212_soft_t            soft = (vt6212_soft_t) arg;
     vertex_hdl_t            vhdl = soft->vt_vhdl;
     vt6212_regs_t            regs = soft->vt_regs;
     cmn_err(CE_CONT, "vt6212 %v: dma done, regs at 0x%X\n", vhdl, regs);
     /*
      * for each buf our hardware has processed,
      *      set buf->b_resid,
      *      call pciio_dmamap_done,
      *      call bioerror() or biodone().
      *
      * XXX - would it be better for buf->b_iodone
      * to be used to get to pciio_dmamap_done?
      */
     /*
      * may want to call pollwakeup.
*/
}
/* ====================================================================
  *          ERROR HANDLING ENTRY POINTS
  */
static int
vt6212_error_handler(void *einfo,
                     int error_code,
                     ioerror_mode_t mode,
                     ioerror_t *ioerror)
{
     vt6212_soft_t            soft = (vt6212_soft_t) einfo;
     vertex_hdl_t            vhdl = soft->vt_vhdl;
#if DEBUG && ERROR_DEBUG
     cmn_err(CE_CONT, "%v: vt6212_error_handler\n", vhdl);
#else
     vhdl = vhdl;
#endif
     /*
      * XXX- there is probably a lot more to do
      * to recover from an error on a real device;
      * experts on this are encouraged to add common
      * things that need to be done into this function.
      */
     ioerror_dump("sample_pciio", error_code, mode, ioerror);
     return IOERROR_HANDLED;
}
/* ====================================================================
  *          SUPPORT ENTRY POINTS
  */
/*
  *    vt6212_halt: called during orderly system
  *      shutdown; no other device driver call will be
  *      made after this one.
  */
void
vt6212_halt(void)
{
     printf("vt6212_halt()\n");
}
/*
  *    vt6212_size: return the size of the device in
  *      "sector" units (multiples of NBPSCTR).
  */
int
vt6212_size(dev_t dev)
{
  vertex_hdl_t            vhdl = dev_to_vhdl(dev);
  vt6212_soft_t            soft = vt6212_soft_get(vhdl);
  return soft->vt_blocks;
}
  /*
  *    vt6212_print: used by the kernel to report an
  *      error detected on a block device.
  */
int
vt6212_print(dev_t dev, char *str)
{
  cmn_err(CE_NOTE, "%V: %s\n", dev, str);
  return 0;
}
