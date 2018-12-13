require 'cosmos/processors/processor'
module Cosmos
  class AltitudeProcessor < Processor
    # @param item_name [Array<String>] The names of the items to mean
    def initialize(temp, press) 
      super(:CONVERTED) # Hard code to work on converted values
      @temperature = temp # Array of the item names
	  @pressure = press
      reset()
    end

    def call(packet, buffer)
      values = []
	  @results = 
    end

    # Reset any state
    def reset
      @results[:MEAN] = []
    end

    # Convert to configuration file string
    def to_config
      "  PROCESSOR #{@name} #{self.class.name.to_s.class_name_to_filename} #{@item_names.join(' ')}\n"
    end
  end
end