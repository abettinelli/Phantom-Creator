function M = ij2RCS(DICOMinfo)

dictionary = 'C:\Program Files\MATLAB\R2020a\toolbox\images\iptformats\dicom-dict.txt';

ImagePosition = DICOMinfo.(images.internal.dicom.lookupActions('0020','0032', dictionary));
ImageOrientation = DICOMinfo.(images.internal.dicom.lookupActions('0020','0037', dictionary));
PixelSpacing = DICOMinfo.(images.internal.dicom.lookupActions('0028','0030', dictionary));

M = zeros(4);

M(1:3,1) = ImageOrientation(1:3)*PixelSpacing(1);
M(1:3,2) = ImageOrientation(4:6)*PixelSpacing(2);
M(1,4) = ImagePosition(1);
M(2,4) = ImagePosition(2);
M(3,4) = ImagePosition(3);
M(4,4) = 1;
