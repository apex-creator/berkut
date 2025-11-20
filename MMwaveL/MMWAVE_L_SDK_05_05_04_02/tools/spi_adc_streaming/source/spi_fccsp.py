import sys, time, ftd2xx as ftd
import numpy as np
import os
import math
import sys
import readchar

######################## Configuration bytestrings ######################################################
FTDI_CFG_60MHZ_SYSCLK = b'\x8A'         # AN108 6.1:   Disable Clk Divide by 5, resulting in 60MHz system clock
FTDI_CFG_NO_ADAPT_CLK = b'\x97'         # AN108 6.10:  Turn Off Adaptive clocking
FTDI_CFG_NO_3PHAS_CLK = b'\x8D'         # AN108 6.4:   Disable 3 Phase Data Clocking
FTDI_CFG_SET_CLK_DIV  = b'\x86'         # AN108 3.8.2: Set clk divisor, [0x86,0xValueL,0xValueH]
FTDI_CFG_NO_LOOPBACK  = b'\x85'         # AN108 3.7.2: Disconnect TDI to TDO for Loopback
FTDI_CFG_SPI_4PIN_CFG = b'\x80\x08\x0B' # AN108 3.6.1: Set Data bits LowByte [0x80,0xValue,0xDirection] (value and direction are bitmasks for FTDI pins)
FTDI_CFG_SPI_WITH_GPIO = b'\x80\x08\x09'# AN108 3.6.1: Set Data bits LowByte [0x80,0xValue,0xDirection] (value and direction are bitmasks for FTDI pins)

######################### Command bytestrings ############################################################
FTDI_CMD_CS_LOW       = b'\x80\x00\x0B' # AN108 3.6.1: Set Data bits LowByte [0x80,0xValue,0xDirection] (value and direction are bitmasks for FTDI pins)
FTDI_CMD_CS_HIGH      = b'\x80\x08\x0B' # AN108 3.6.1: Set Data bits LowByte [0x80,0xValue,0xDirection] (value and direction are bitmasks for FTDI pins)
FTDI_CMD_WRITE_BYTES  = b'\x11'         # AN108 3.3.2: Clock Data Bytes Out on -ve clock edge MSB first (no read) [0x11,LengthL,LengthH,byte0,...,byteN]
FTDI_CMD_READ_BYTES   = b'\x20'         # AN108 3.3.5: Clock Data Bytes In on +ve clock edge MSB first (no write) [0x20,LengthL,LengthH]
FTDI_CMD_RW_BYTES     = b'\x31'         # AN108 3.3.9: Clock Data Bytes In on +ve and Out on -ve MSB first [0x20,LengthL,LengthH,byte0,...,byteN]
FTDI_CMD_READ_BITS    = b'\x81'         # AN108 3.6.3: Read Data bits LowByte, read the current state of the first 8 pins and send back 1 byte

FTDI_MAX_SIZE = 65536

######################### Write the FTDI command bytestring #############################################
#   buf     - can be a bytestring (ex. b'\x00\x01\x02') or a list of ints (ex [0,1,2])
#   len     - from [1,65536] 
#   returns - the FTDI command bytestring
def spi_write_byte(buf, len):
    if (len > 0x10000 or len < 1):
        raise ValueError('len cannot be greater than 65536 or less than 1')
    
    # According to AN108 3.3.2, length of 0x0000 = 1 byte, 0x0001 = 2 bytes, etc., hence 'len - 1'
    little_end_len = int(len - 1).to_bytes(2,'little') 
    b_str = bytes(buf[:len])
    return FTDI_CMD_CS_LOW + FTDI_CMD_WRITE_BYTES + little_end_len + b_str + FTDI_CMD_CS_HIGH
######################## Read the FTDI command bytestring ###############################################
#   len     - from [1,65536]
#   returns - the FTDI command bytestring
def spi_read_bytestring(len):
    if (len > 0x10000 or len < 1):
        raise ValueError('len cannot be greater than 65536 or less than 1')

    # According to AN108 3.3.2, length of 0x0000 = 1 byte, 0x0001 = 2 bytes, etc., hence 'len - 1'
    little_end_len = int(len - 1).to_bytes(2,'little') 
    return FTDI_CMD_CS_LOW + FTDI_CMD_READ_BYTES + little_end_len + FTDI_CMD_CS_HIGH

######################## FTDI WRITE #####################################################################
# Max - 0xFFFF bytes
# This function handles CS (active low) automatically.
def spi_write(handle, buf, len):
    cmd_str = spi_write_byte(buf,len)
    handle.write(cmd_str)

######################## FTDI READ #####################################################################
# Max - 0x8000 
# This function handles CS (active low) automatically.
def spi_read(handle, len):
    cmd_str = spi_read_bytestring(len)
    handle.write(cmd_str)
    res = list(handle.read(len))
    return res
######################## FTDI READ BINARY ###############################################################
# Max - 0x8000 
# This function handles CS (active low) automatically.
# returns binary
def spi_read_bin(handle, len):
    cmd_str = spi_read_bytestring(len)
    handle.write(cmd_str)
    return handle.read(len)

######################## SET CLOCK ######################################################################
# Write out an FTDI command to set clock speed from a 60MHz system clock
def set_clk(handle, hz):
    if (hz > 30000000):
        raise ValueError('max SCK rate is 30MHz')

    div = int((60000000 / (hz * 2)) - 1)        # Set SPI clock
    cmd_str = bytes((0x86, div%256, div//256))  # TCK period = 60MHz / (( 1 +[ (0xValueH * 256) OR 0xValueL] ) * 2) 
    handle.write(cmd_str)

######################## FTDI PARAM INIT ################################################################
# Initialize an FTDI device via the FTD2XX library wrapper around the D2XX driver
def set_device(handle, clk_speed=1000000, usb_tx_req_size=65535, rw_timeout=200000, latency_timer=2):
    handle.resetDevice()

    # Purge USB receive buffer by reading out all old data from FT4232 rx buffer
    (rx_bytes, tx_bytes, event_status) = handle.getStatus()
    if rx_bytes > 0:
        handle.read(rx_bytes)

    handle.setUSBParameters(usb_tx_req_size, usb_tx_req_size) # Set USB request transfer size, default is 4096 bytes; TODO experiment with this
    handle.setChars(False, 0, False, 0) # Disable event and error characters
    handle.setTimeouts(rw_timeout,rw_timeout) # Set read and write timeouts
    handle.setLatencyTimer(latency_timer) # TODO experiment with this
    handle.setBitMode(0,0) # Reset the MPSSE controller
    handle.setBitMode(0,2) # Enable MPSSE mode
    time.sleep(0.050)
    handle.write(FTDI_CFG_60MHZ_SYSCLK)
    handle.write(FTDI_CFG_NO_ADAPT_CLK)
    handle.write(FTDI_CFG_NO_3PHAS_CLK)
    handle.write(FTDI_CFG_SPI_4PIN_CFG)
    set_clk(handle, clk_speed) # Set clock frequency ; TODO experiment with this
    time.sleep(0.020)
    handle.write(FTDI_CFG_NO_LOOPBACK)
    time.sleep(0.030)

########################## GLOBALS ######################################################################
res1_temp=[] 
fin_list=[]
count_parts=0
device = input("\"1\" for AOP\n\"2\" for FCCSP \nEnter Device: \n")
adc_samp = input("Enter no of adc samples: \n")
chirps_per_burst = input("Enter no of chirps per burst: \n")
bursts_per_frame = input("Enter no of bursts per frame: \n")
frames = input("Enter no of frames: \n")
rx_antenna= input("Enter no of rx antennas: \n")
# format= input("\n\"1\" for 32 bit Hex\n\"2\" for 16 bit Signed Int \nEnter Format to print: \n")
format=2
device = int(device)
adc_samp = int(adc_samp)
chirps_per_burst = int(chirps_per_burst)
bursts_per_frame = int(bursts_per_frame)
frames = int(frames)
rx_antenna = int(rx_antenna)
format= int(format)
size_rd = 2*adc_samp*chirps_per_burst*bursts_per_frame*rx_antenna
print(size_rd)

######################### CONVERTS TO HEX NUMBERS ########################################################
def parser(lis):
    global count_parts
    stri=0
    temp=0
    count_check=0
    for i in range(len(lis)):
        temp= lis[i]*pow(256,(63-count_check)%4)
        count_check+=1
        stri= stri+temp
        if((i !=0 ) and ((i+1)%4==0)):
            fin_list.append(hex(stri))
            hexstri= "0x{:08X}".format(stri)
            temp=0
            stri=0
            count_check=0

######################## SPLITS NUMBER FOR INT 16 ##########################################################
def padhexa_form(s):
    return s[2:].zfill(8)

######################## PADS HEX NUMBERS ##################################################################
def padhexa(s):
    return '0x' + s[2:].zfill(8)

# def splitfile():
#     count_parts=0
#     N=5
#     sizeoflist=len(fin_list)/N
#     for i in range(len(fin_list)):
#         hexstri= padhexa(fin_list[i])
#         count_parts=math.ceil(i/sizeoflist)
#         if(i==0):
#             count_parts=1
#         filname="part"+str(count_parts)+".txt"
#         f=open(filname, 'a')
#         f.write(hexstri+"\n")

######################## TWOS COMPLEMENT CALC ############################################################### 
def twos_complement(hexstr, bits):
    value = int(hexstr, 16)
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value

######################## WRITE HEX TO FILE ##################################################################
def printfile():
    filname="adcdata"+".txt"
    f=open(filname, 'a+')
    for i in range(len(fin_list)):
        fin_list[i]= padhexa(fin_list[i])
        f.write(fin_list[i]+'\n')

######################## WRITE INT16 TO FILE #################################################################    
def printfile_format(f):
    for i in range(len(fin_list)):
        fin_list[i]= padhexa_form(fin_list[i])
        hexstrx=fin_list[i]
        hexstry=fin_list[i]
        hexstrx=hexstrx[0:4]
        hexstry=hexstry[4:]
        hexstrx= twos_complement(hexstrx,16)
        hexstry= twos_complement(hexstry,16)
        f.write(str(hexstry)+'\n')
        f.write(str(hexstrx)+'\n')

######################## READ GPIO VAL #######################################################################
# input - FTDI Handle    
# return - 8 bit uint value
def read_gpio(handle):
    handle.write(b'\x81')
    res= handle.read(1)
    value = int.from_bytes(res, 'big')
    return value

######################## FILE EXISTS CHECK ####################################################################
if os.path.exists("adcdata.txt"):
  os.remove("adcdata.txt")
  
######################## DRIVER CODE FOR FCCSP ################################################################
def spi_fccsp():
    filname="adcdata"+".txt"
    f=open(filname, 'a+')
    dev = ftd.open(0)
    print("FTDI device opened:")
    print(dev.getDeviceInfo())
    print('Starting write loop...')
    set_device(dev, 15000000, latency_timer=1)
    count=0
    while(count<frames):
        count+=1
        value= read_gpio(dev)
        size_back=size_rd
        size_temp=size_rd
        while(size_back >0):
            if(size_back>FTDI_MAX_SIZE):
                size_temp = FTDI_MAX_SIZE
            else:
                size_temp = size_back
            size_back=size_back-size_temp
            while((value & 0x10) != 0 ):
                value= read_gpio(dev)
                # print(value)
            res1 = spi_read(dev,size_temp)
            res1_temp.extend(res1)
    parser(res1_temp)
    if(format==1):
        printfile()
    else:
        printfile_format(f) 
    print("\n\nPress Any Key To Exit")
    k = readchar.readchar()
    
######################## DRIVER CODE FOR AOP ################################################################
def spi_aop():
    filname="adcdata"+".txt"
    f=open(filname, 'a+')
    dev = ftd.open(0)
    dev1 = ftd.open(1)
    print("FTDI device opened:")
    print(dev.getDeviceInfo())
    print('Starting write loop...')
    set_device(dev, 15000000, latency_timer=1)
    set_device(dev1, 15000000, latency_timer=1)

    count=0
    while(count<frames):
        count+=1
        value= read_gpio(dev)
        size_back=size_rd
        size_temp=size_rd
        while(size_back >0):
            if(size_back>FTDI_MAX_SIZE):
                size_temp = FTDI_MAX_SIZE
            else:
                size_temp = size_back
            size_back=size_back-size_temp
            while((value & 0xA0) != 0 ):
                value= read_gpio(dev1)
                # print(value)
            res1 = spi_read(dev,size_temp)
            res1_temp.extend(res1)
    parser(res1_temp)
    if(format==1):
        printfile()
    else:
        printfile_format(f) 
    print("\n\nPress Any Key To Exit")
    k = readchar.readchar()

###################### DRIVER CODE FOR SCRIPT #################################################################    
if (device==1):
    spi_aop()
if (device==2):
    spi_fccsp()