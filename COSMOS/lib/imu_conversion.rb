# Implements a conversion factor based on the MPXH6400A pressure sensor spec
# The conversion assumes in input voltage of 5.0V and outputs in kPa

require 'cosmos/conversions/conversion'

module Cosmos
  class ImuConversion < Conversion
    def call(value, packet, buffer)
		  return (value*2.0) / 32768.0	
    end
  end
end