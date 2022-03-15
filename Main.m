function Main()
%%%Doc Starts%%%
% -Description:
%	Place in the Image folder the dicom files
%	The output RTstruct is saved in the specified folder
%
% -Parameters:
%	None
%
% -Revision:
%	2022-03-14: The method is packed for sharing.
%	2021-04-10: The method is implemented.
%
% -Author:
%	Andrea Bettinelli

input_folder = '.\Image';
output_folder = '.\Output';             

% Load utilitiy functions
addpath(fullfile('.','Utilities'))

% Load Dicom Image and headers
[image, imgHeaders, RTSTRUCT] = loadDicomImage(input_folder);

% Create the custom binary mask
image_ROI = ROIpositioning(image);
if ~any(image_ROI.Mask(:))
   error('No ROIs where placed on the image') 
end
load(fullfile('.','Utilities','Temp','mask_templates_gui.mat'))

% Convert RTSTRUCT info to dicomContours object
contourSet = create_empty_contourSet(RTSTRUCT);

% Match the image mask to templates, convert it to contours, add to contourOut
contourSet = correspondence_mask_add(mask_templates, image_ROI, imgHeaders, contourSet);

% Plot Contour Set
contourSet.ROIs

figure()
h = plotContour(contourSet);
axis equal
for n = 1:size(contourSet.ROIs,1)
    coord = mean(contourSet.ROIs.ContourData{n}{1});
    text(coord(:,1),coord(:,2),coord(:,3)+5,strrep(erase(contourSet.ROIs.Name{n},'ROI_'), '_', ' '),'HorizontalAlignment', 'center','Parent',h(n))
end

% Convert dicomContours object to RT-STRUCT info
info_matlab_rtstruct = convertToInfo(contourSet);

% Save RT-STRUCT file
tic, dicomwrite([],fullfile(output_folder, 'RTSS_phantom.dcm'),info_matlab_rtstruct,'CreateMode','copy');
disp(['Writing Output file required ' num2str(toc) ' seconds'])

% Remove from path "Utilities" folder

rmpath(fullfile('.','Utilities'))