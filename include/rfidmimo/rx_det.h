/* -*- c++ -*- */
/* 
 * Copyright 2021 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */


#ifndef INCLUDED_RFIDMIMO_RX_DET_H
#define INCLUDED_RFIDMIMO_RX_DET_H

#include <rfidmimo/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace rfidmimo {

    /*!
     * \brief <+description of block+>
     * \ingroup rfidmimo
     *
     */
    class RFIDMIMO_API rx_det : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<rx_det> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of rfidmimo::rx_det.
       *
       * To avoid accidental use of raw pointers, rfidmimo::rx_det's
       * constructor is in a private implementation
       * class. rfidmimo::rx_det::make is the public interface for
       * creating new instances.
       */
      static sptr make(int num_ant, float samp_rate);
    };

  } // namespace rfidmimo
} // namespace gr

#endif /* INCLUDED_RFIDMIMO_RX_DET_H */

