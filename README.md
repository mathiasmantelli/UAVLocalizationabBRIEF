# UAVLocalizationabBRIEF

How to setup this framework.

What you have to install: 
* [OpenCV 3.2.0](https://www.samontab.com/web/2017/06/installing-opencv-3-2-0-with-contrib-modules-in-ubuntu-16-04-lts/)
* [Aria 2.7.2](https://firebasestorage.googleapis.com/v0/b/mathiassite-987b8.appspot.com/o/ARIA-2.7.2.tgz?alt=media&token=ff2c283d-f6db-4066-a2ec-8128f97400ed) - If you are using Ubuntu 16.04, you have to delete the line 623 from "Makefile" file.
* [QtCreator](https://www.qt.io/download-qt-installer?hsCtaTracking=99d9dd4f-5681-48d2-b096-470725510d34%7C074ddad0-fdef-4e53-8aa8-5e8a876d6ab4)

What you have to download: 
* [Arroio do Meio dataset](https://zenodo.org/record/1244296#.X2zT13VKhBQ)
* [Porto Alegre dataset](https://zenodo.org/record/1244314#.X20Jx3VKhBQ)

Where to paste what:
After downloading everything, you have to create the following structure. The _italic itens_ you have to create by yourself:
Datasets:
  ↳Arroio do Meio<br>
    &nbsp;↳Flight1<br>
    &nbsp;↳Flight2<br>
    &nbsp;↳Maps<br>
    &nbsp;↳_Output_<br>
       &nbsp;&nbsp;↳_globalmap_<br>
    &nbsp;↳_flight1.txt_<br>
    &nbsp;↳_flight2.txt_<br>
  ↳Porto Alegre<br>
    &nbsp;↳Flight3<br>
    &nbsp;↳Maps<br>      
    &nbsp;↳_Output_<br>
       &nbsp;&nbsp;↳_globalmap_<br>  
    &nbsp;↳_flight1.txt_<br>
    
The "Flight*" folders contain all images from each trajectory; 
The "Maps" folders contain the satellite images extracted from Google Earth. One of these maps must be called "globalmap.png", because that is the file that the framework looks for; 
The "_Output_" folders save the output files generated by the framework. The "_globalmap_" is the name of the map that is used as input;
The "_flight*.txt_" files contain the address of the images within the respective folders. For example, the "_flight1.txt_" contains the addresses of all images within Flight1

To execute the framework using QtCreator, you have to open the project, and go to "Projects - Run" and in "Command line arguments", you have to set the inputs, and it is like this:
-e <Map_folder_address> -t <flight_txt_file_address> -s BRIEF diff-cie2000 10 -bp 256 -blt 0.5 -bmt 4 -bm 10 -o <globalmap_output_folder_address> 1 2

The framework is supposed to compute the Visual Odometry when this information is not available beforehand. However, for a reason that I'm still not sure yet, it is not computing such information. I have to figure this out asap. 
There are other instructions I have to share with you, please let me know when you finish this part. 
    
    
