//****************************************************************************
// Copyright (C) 2006-2007  PEAK System-Technik GmbH
//
// linux@peak-system.com
// www.peak-system.com
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// Maintainer(s): Klaus Hitschler (klaus.hitschler@gmx.de)
//
// Major contributions by:
//                Edouard Tisserant (edouard.tisserant@lolitech.fr) XENOMAI
//                Laurent Bessard   (laurent.bessard@lolitech.fr)   XENOMAI
//                Oliver Hartkopp   (oliver.hartkopp@volkswagen.de) socketCAN
//                     
//****************************************************************************

//***************************************************************************
//
// system dependend parts to handle pcan-pccard
// special code for kernels greater and equal than 2.6.17
//
// $Id: pcan_pccard-2.6.17.c 447 2007-01-28 14:05:50Z khitschler $
//
//****************************************************************************


//****************************************************************************
// CODE

//****************************************************************************
// is called at CARD_REMOVAL
static void pccard_plugout(struct pcmcia_device *link)
{
  PCAN_PCCARD *card = link->priv;

  DPRINTK(KERN_DEBUG "%s: pccard_plugout(0x%p)\n", DEVICE_NAME, link);

  pccard_release_all_devices(card);
  pcmcia_disable_device(link);
}

//****************************************************************************
// is called at CARD_INSERTION or startup with already inserted card
static int pccard_plugin(struct pcmcia_device *link)
{
  PCAN_PCCARD     *card = link->priv;
  tuple_t         tuple;
  cisdata_t       buf[CISTPL_END + 1];
  cisparse_t      parse;
  config_info_t   conf;
  cistpl_vers_1_t *cv;
  int             last_ret;
  int             last_fn;
  
  DPRINTK(KERN_DEBUG "%s: pccard_plugin(0x%p)\n", DEVICE_NAME, link);
  
  // show manufacturer string in log
  CS_PREPARE(CISTPL_VERS_1);
  CS_CHECK(GetFirstTuple, pcmcia_get_first_tuple(link, &tuple));
  CS_CHECK(GetTupleData, pcmcia_get_tuple_data(link, &tuple));
  CS_CHECK(ParseTuple, pcmcia_parse_tuple(link, &tuple, &parse));
  cv = &parse.version_1;
  printk(KERN_INFO "%s: %s %s %s %s\n", DEVICE_NAME, &cv->str[cv->ofs[0]], &cv->str[cv->ofs[1]], 
                                                     &cv->str[cv->ofs[2]], &cv->str[cv->ofs[3]]);

  // specify what to request
  CS_PREPARE(CISTPL_CONFIG);
  CS_CHECK(GetFirstTuple, pcmcia_get_first_tuple(link, &tuple));
  CS_CHECK(GetTupleData,  pcmcia_get_tuple_data(link, &tuple));
  CS_CHECK(ParseTuple,    pcmcia_parse_tuple(link, &tuple, &parse));
  
  link->conf.ConfigBase = parse.config.base;
  link->conf.Present    = parse.config.rmask[0];
  
  CS_CHECK(GetConfigurationInfo, pcmcia_get_configuration_info(link, &conf));
  
  tuple.DesiredTuple = CISTPL_CFTABLE_ENTRY;
  tuple.Attributes   = 0;
  CS_CHECK(GetFirstTuple, pcmcia_get_first_tuple(link, &tuple));
  while (1) 
  {
    cistpl_cftable_entry_t *cfg = &parse.cftable_entry;
  
    if (pcmcia_get_tuple_data(link, &tuple) != 0 || pcmcia_parse_tuple(link, &tuple, &parse) != 0)
      goto next_entry;

    if (cfg->io.nwin > 0) 
    {
      cistpl_io_t *io = &cfg->io;
      
      link->conf.Attributes  = CONF_ENABLE_IRQ;
      link->conf.IntType     = INT_MEMORY_AND_IO;
      link->conf.ConfigIndex = cfg->index;
      link->conf.Present     = PRESENT_OPTION | PRESENT_STATUS;
      
      link->irq.Attributes   = IRQ_TYPE_EXCLUSIVE;
      link->irq.IRQInfo1     = cfg->irq.IRQInfo1;
      link->irq.IRQInfo2     = cfg->irq.IRQInfo2;
      link->irq.Handler      = NULL; // we use our own handler
      
      link->io.BasePort1     = io->win[0].base;
      link->io.NumPorts1     = io->win[0].len;
      link->io.Attributes1   = IO_DATA_PATH_WIDTH_8; // only this kind of access is yet supported
      link->io.IOAddrLines   = io->flags &  CISTPL_IO_LINES_MASK;
      
      if (pcmcia_request_io(link, &link->io) == 0)
        break;
    }

    next_entry:
    CS_CHECK(GetNextTuple, pcmcia_get_next_tuple(link, &tuple));
  }
  
  CS_CHECK(RequestIRQ, pcmcia_request_irq(link, &link->irq));
  CS_CHECK(RequestConfiguration, pcmcia_request_configuration(link, &link->conf));

  DPRINTK(KERN_DEBUG "%s: pccard found: base1=0x%04x, size=%d, irq=%d\n", DEVICE_NAME, link->io.BasePort1, link->io.NumPorts1, link->irq.AssignedIRQ);

  // init (cardmgr) devices associated with that card (is that necessary?)
  card->node.major = pcan_drv.nMajor;
  card->node.minor = PCCARD_MINOR_BASE;
  strcpy(card->node.dev_name, DEVICE_NAME);
  link->dev_node = &card->node;
  
  // create device descriptors associated with the card - get relevant parts to get independend from dev_link_t
  card->basePort  = link->io.BasePort1; 
  card->commonIrq = link->irq.AssignedIRQ;
  last_ret = pccard_create_all_devices(card);
  if (last_ret)
    goto fail;
  
  return 0;
    
  cs_failed:
  cs_error(link, last_fn, last_ret);
  
  fail:
  pccard_plugout(link);
  
  DPRINTK(KERN_DEBUG "%s: pccard_plugin() failed!\n", DEVICE_NAME);
  return -ENODEV;
}

//****************************************************************************
// probe entry
static int pccard_probe(struct pcmcia_device *link)
{
  PCAN_PCCARD *card;

  DPRINTK(KERN_DEBUG "%s: pccard_probe()\n", DEVICE_NAME);

  /* Create new PCAN_PCCARD device */
  card = kmalloc(sizeof(*card), GFP_KERNEL);
  DPRINTK(KERN_DEBUG "%s: kmalloc() = 0x%p\n", DEVICE_NAME, card);
  if (card == NULL)
  {
    DPRINTK(KERN_DEBUG "%s: can't allocate card memory\n", DEVICE_NAME);
    return -ENOMEM;
  }

  memset(card, 0, sizeof(*card));
  link->priv    = card;
  card->pcc_dev = link;

  return pccard_plugin(link);
}

//****************************************************************************
// detach entry
static void pccard_detach(struct pcmcia_device *link)
{
  DPRINTK(KERN_DEBUG "%s: pccard_detach()\n", DEVICE_NAME);

  pccard_plugout(link);

  DPRINTK(KERN_DEBUG "%s: kfree(PCAN_PCCARD=0x%p)\n", DEVICE_NAME, link->priv);
  kfree(link->priv);
  link->priv = NULL;
}

//****************************************************************************
// suspend entry
static int pccard_suspend(struct pcmcia_device *link)
{
  DPRINTK(KERN_DEBUG "%s: pccard_suspend()\n", DEVICE_NAME);

  return 0;
}

//****************************************************************************
// resume entry
static int pccard_resume(struct pcmcia_device *link)
{
  DPRINTK(KERN_DEBUG "%s: pccard_resume()\n", DEVICE_NAME);

  return 0;
}

// fin
