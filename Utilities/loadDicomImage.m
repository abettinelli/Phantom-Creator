function [image, headers, RTstruct] = loadDicomImage(input_folder)

[headers, datas, ~, RTstruct] = loadCellDicom(input_folder, true);

image.Volume = cat(3,datas{:});
image.Mask = logical(uint8(zeros(size(image.Volume))));
image.XPixDim = headers{1}.PixelSpacing(1);
image.YPixDim = headers{1}.PixelSpacing(2);
image.ZPixDim = headers{1}.SliceThickness;
image.XDim = size(image.Volume,2);
image.YDim = size(image.Volume,1);
image.ZDim = size(image.Volume,3);
image.ImagePositionPatient = headers{1}.ImagePositionPatient;
image.XStart = headers{1}.ImagePositionPatient(1);
image.YStart = headers{1}.ImagePositionPatient(2);
image.ZStart = headers{1}.ImagePositionPatient(3);
image.ImageOrientationPatient = headers{1}.ImageOrientationPatient;
image.PixelSpacing = [headers{1}.PixelSpacing; headers{1}.SliceThickness];

end

function [imgHeaders, imgDatas, imgInfos, RTstruct] = loadCellDicom(path, varargin)

temp = dir2(path);
reference_file = fullfile(temp(1).folder, temp(1).name);

if isempty(varargin)
    flag_norm = true;
else
    flag_norm = varargin{1};
end

reference_header = dicominfo(reference_file);
reference_studyInstanceUID = reference_header.StudyInstanceUID;
imagedir = fileparts(reference_file);

imagefiles = dir([imagedir filesep '*']);
imagefiles = imagefiles(~[imagefiles.isdir]);

% Image Header
imgHeaders = cell(length(imagefiles),1);
imgDatas = cell(length(imagefiles),1);
imgInfos = cell(length(imagefiles),1);

sliceno = NaN(1,length(imagefiles));
for i  = 1:length(imagefiles)
        info = dicominfo(fullfile(imagedir, imagefiles(i).name)); %, 'UseDictionaryVR', true);
        
        % Skip files from other studies and DICOM-RT files.
        if strcmp(info.StudyInstanceUID, reference_studyInstanceUID ) && isempty(regexpi(info.Modality, '^RT.*'))
            sliceno(i) = info.InstanceNumber+1;
            
            % Header
            imgHeaders{info.InstanceNumber+1} = info;
            
            % Image
            curr_imgData = dicomread(fullfile(imagedir, imagefiles(i).name));
            if flag_norm
                try info.RescaleSlope; catch, info.RescaleSlope = 1; end
                try info.RescaleIntercept; catch, info.RescaleIntercept = 0; end
                imgDatas{info.InstanceNumber+1} = double(curr_imgData)*info.RescaleSlope+info.RescaleIntercept;
                
            else
                imgDatas{info.InstanceNumber+1} = dicomread(fullfile(imagedir, imagefiles(i).name));
            end
            imgInfos{info.InstanceNumber+1}.OriginalClass = class(curr_imgData);
        end
        
        % RTSTRUCT file
        if strcmp(info.StudyInstanceUID, reference_studyInstanceUID ) && ~isempty(regexpi(info.Modality, '^RT.*'))
            RTstruct = info;
        end
end

imgHeaders = imgHeaders(min(sliceno):max(sliceno));
imgDatas = imgDatas(min(sliceno):max(sliceno));
imgInfos = imgInfos(min(sliceno):max(sliceno));

if ~exist('RTstruct','var')
    error('An RTSTRUCT DICOM file should be present in the "Image" folder')
end

end