clear all;
close all;

[sonar, libName] = loadBvtSdk('FILE','/home/dnad/Development/sonar/900Linija03.son');
%[sonar, libName] = loadBvtSdk('FILE','/home/dnad/Development/sonar/p450stuff.son');


head = libpointer;
calllib(libName, 'BVTSonar_GetHead', sonar, 0, head);
calllib(libName, 'BVTHead_SetImageType' , head , 1);
calllib(libName, 'BVTHead_SetImageRes' , head , 1);

% obtain parameters in sonar file
pingCount = calllib(libName,'BVTHead_GetPingCount',head);

i=460;
while 1
    %Get the magnitude image
    ping = libpointer;
    calllib(libName, 'BVTHead_GetPing', head, i , ping);
    mag_img = libpointer;
    calllib('libbvtsdk', 'BVTPing_GetImage', ping, mag_img);
    
    %Get image size
    height = calllib('libbvtsdk','BVTMagImage_GetHeight', mag_img);
    width = calllib('libbvtsdk', 'BVTMagImage_GetWidth', mag_img);
    
    %Get bits and convert to array
    img_ptr = libpointer('uint16Ptr',calllib('libbvtsdk', 'BVTMagImage_GetBits', mag_img));
    img_ptr.reshape(width,height);
    img = img_ptr.Value;
    
    %Convert and scale
    magnitude = 20*log10(double(img)');
    %magnitude = magnitude/max(max(magnitude));
    
    magf = magnitude; magf = (magf-26.9359)/2.8903; magf(magf<0) = 0;
    figure(1);imagesc(magnitude);colormap('jet');
    figure(2);imagesc(magf);colormap('jet');
    imgf = box_car(magnitude);
    figure(3);imagesc(imgf);colormap('jet');
    drawnow;
    
    %Clear all
    calllib(libName,'BVTMagImage_Destroy', mag_img);
    calllib(libName,'BVTPing_Destroy', ping);        
    
    i=mod((i+1),pingCount);
end
