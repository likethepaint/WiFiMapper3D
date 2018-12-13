# Implements a conversion factor based on the MPXH6400A pressure sensor spec
# The conversion assumes in input voltage of 5.0V and outputs in kPa

require 'cosmos/conversions/conversion'

module Cosmos
  class TempConversion < Conversion
    def call(value, packet, buffer)
		  return (value / 16.0) + 27.5	
    end
  end
end