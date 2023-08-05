import pyocd

# This example applies to the NXP i.MX RT10x0 devices.

# Unlike the previous example, the board argument is excluded here.
def will_connect():
    # Look up the external flash memory region.
    #extFlash = target.memory_map.get_first_matching_region(name="flexspi")
    flash = target.memory.STM32H7x_2048
    # Set the path to an .FLM flash algorithm.
    #extFlash.flm = "..\STM32H7x_2048.FLM"
    print("Hello word")
