
To create Integrity bin:

Integrity_generator.exe [symmterickey.txt] [salt.txt] [info.txt] [integrity.bin] app [app_rprc.bin] rfs [rfs_rprc.bin]

-----------------------------------------------------------------------------------------------------------------------

Metaimage creation:

multicore_image_generator.exe LE [BOOT_MODE] [BOOT_VECTOR] [SH_MEM_CONFIG] metaimage.bin  0xab130000 [integrity.bin] 0x35510000 [app_rprc.bin] 0xb5510000 [rfs_rprc.bin]


-----------------------------------------------------------------------------------------------------------------------

Convert Text File with key to Binary format (this is used to convert text file containing the key to binary format as keys need to be sent in binary format ): 

txt2bin.exe [symmterickey.txt]

