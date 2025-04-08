# Information Backup

The information that needs to be backed up, for now I think there are two types.

The temperature and its duration times
The PID control constants.
The temperature curve has 4 important points

![image](https://github.com/user-attachments/assets/55caf553-f7c7-4931-adda-a696af2e89e8)

The constants are 32-bit float type, so 4 bytes of each are needed.

So, we have a table that should hold that information and it looks like this image.

![image](https://github.com/user-attachments/assets/f3544a12-e841-4dfb-b984-db598ded8929)

The last four bytes correspond to the 36-bit CRC value of all the previous values. This way, it can be determined that everything stored in the 28 bytes is something useful and logical.

I think it will stay like this for the moment, I was thinking that possibly there should be "more temperature curves" with their important points and maybe different constants for other PID controllers, but for the moment I will leave it like this.

