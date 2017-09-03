# 1. Crawl on Key Knowledge
1. Telemetry Coordinates  
*Euler angels*; *naval and aeronautical position*  
![Example](../Ref/TelemetryCoordinates.JPG)  
2. Python coding
3. Record data from simulator trainning mode for IMG & log in the folder 'record_data'
4. Choose one picture from record data folder [robocam_2017_08_26_17_59_22_096.jpg](record_data/IMG/robocam_2017_08_26_17_59_22_096.jpg) for later coding Example  
![CodingExpPict](record_data/IMG/robocam_2017_08_26_17_59_22_096.jpg)  
5. Basic knowledge about deal with image  
 - Matrix concepts to deal with image  
  e.g.  
$$
\left[
\begin{matrix}
1 & 0 & t_x \\
0 & 1 & t_y
\end{matrix}
\right]
$$
 - *image pixels*: 8 bits information for each pix of each channel (0,255)
 - *image channel*: RGB; BGR; Gray
 - *libraries for image* matplotlib; cv2; Mahotas; PIL etc.
 - *Coordinates for image*: raw image Coordinates; Rover-Centric Coordinates; map to World Coordinates; polar Coordinates  
 - *layout method by plt.subplot(m, n, p)*  
6. Flowchart on the project of 'Search & sample return'  
