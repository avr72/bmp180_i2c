# bmp180_i2c
Beaglebone black and pressure sensor BMP180 communication using I2C, C code cross compiled in Eclipse

The purpose of the code is to demonstrate how to use I2C device on beaglebone board.  

To cross compile a C application for your beagle board running debian, you need to first install the cross compile toolchain in your host system(I use Ubuntu). You can refer to http://www.acmesystems.it/arm9_toolchain for installing the ARM cross compiler toolchain on your Linux Ubuntu PC.  

To create the project in Eclipse  
1. File->New->C Project  
2. In project type, select Cross-Compile Project and toolchains as Cross GCC. Give a project name "BMP180_I2C". Hit Next  
3. In the tool command prefix, fill in "arm-linux-gnueabihf-" without the quotation mark. In the tool command path, fill in "/usr/bin", also without the quotation mark. Hit finish. Now you are ready to write your source code.  

The executable can be conveniently tested in the beagleboard using sshfs following these steps:  
1. log into your beagleboard  
   ssh -X debian@172.27.35.4  
2. create a folder named tmp  
   mkdir ~/tmp  
3. map the eclipse workspace folder to the created folder. 2222 is the forwarded port in my ubuntu virtualbox.  
   sshfs yshen@172.27.35.1:/home/yshen/code/eworkspace /home/debian/tmp/ -p2222  
4. you may need to add your username to the fuse group in /etc/group in order to mount without root priviledge  
5. to unmount, run the following  
   fusermount -u /home/debian/tmp  
6. go to the folder and test/debug the executable  
   cd ~/tmp/BMP180_I2C/Debug  
   gdb BMP180_I2C  
