function [contourData, referencedUID] = binaryToContourSet(imageheaders, imagemask)

% For each ROI
[~,idx] = max(sum(squeeze(sum(imagemask,1)),1));
n_voxel_per_slice = sum(squeeze(sum(imagemask,1)),1);
contourData = cell(0,1);
referencedUID(1).SOPClassUID = [];
referencedUID(1).SOPInstanceUID = [];
SE_square = logical([true,true,true;true,true,true;true,true,true]);
SE_disk = logical([false,true,false;true,true,true;false,true,false]);

counter = 1;
% For each slice
for i = 1:size(imagemask,3)
    if n_voxel_per_slice(i) == 0    % it's empty
        continue
    else                            % crop image around ROI
        [curr_imagemask,i_bk,j_bk] = cropslice(imagemask(:,:,i));
    end
    M = ij2RCS(imageheaders{i});

    slice1 = zeros(size(curr_imagemask(:,:))*2+1+4);
    slice1(4:2:end-2, 4:2:end-2) = curr_imagemask(:,:);
    slice2 = imdilate(slice1, SE_square);
    slice3 = imopen(slice2, SE_disk);
    
    slice_full= imfill(slice3);
    slice_holes = slice_full-slice3;
    slice_holes = imdilate(slice_holes,SE_disk);
    
    B = bwboundaries(slice_full);
    Bh = bwboundaries(slice_holes);
    
    N = size(B,1)+size(Bh,1);
    
    % For each contour in B
    for n=1:size(B,1)
        % Adjust Coordinates
        B_curr = flip((B{n,1}-4)./2+[i_bk-1 j_bk-1],2);                     %+[i_bk-1 j_bk-1] realign
        B_curr = padarray(padarray(B_curr', 1, 0,'post'), 1, 1,'post');
        temp = (M*B_curr)';
        B_mat = temp(:,1:3);
        B_mat = simplify_contour(B_mat);
        
        contourData{counter,:} = B_mat;
        referencedUID(counter).SOPClassUID = imageheaders{i, 1}.SOPClassUID;
        referencedUID(counter).SOPInstanceUID = imageheaders{i, 1}.SOPInstanceUID;
        counter = counter+1;
    end
    
    for n=1:size(Bh,1)
        % Adjust Coordinates
        Bh_curr = flip((Bh{n,1}-4)./2+[i_bk-1 j_bk-1],2);                   %+[i_bk-1 j_bk-1] realign
        Bh_curr = padarray(padarray(Bh_curr', 1, 0,'post'), 1, 1,'post');
        temp = (M*Bh_curr)';
        Bh_mat = temp(:,1:3);
        Bh_mat = simplify_contour(Bh_mat);
        
        contourData{counter,:} = flip(Bh_mat);
        referencedUID(counter).SOPClassUID = imageheaders{i, 1}.SOPClassUID;
        referencedUID(counter).SOPInstanceUID = imageheaders{i, 1}.SOPInstanceUID;
        counter = counter+1;
    end
end
flag = cellfun(@isempty, contourData);
contourData(flag,:) = [];
referencedUID(flag) = [];
end

function [img, i, j] = cropslice(img)

sum_cols = sum(img,1)~=0;
sum_rows = sum(img,2)~=0;

idx_cols = find(sum_cols);
idx_rows = find(sum_rows);
i = idx_rows(1);
j = idx_cols(1);
img = img(i:idx_rows(end), j:idx_cols(end));
end

function contourDataOut = simplify_contour(contourData)

% First dimension
id1 = [1; diff(contourData(:,1))];
id2 = [diff(contourData(:,1)); 1];
idx =  (id1 == 0) & (id2 == 0);

contourDataTemp = contourData;
contourDataTemp = contourDataTemp(idx == 0,:);

% Secon dimension
id1 = [1; diff(contourDataTemp(:,2))];
id2 = [diff(contourDataTemp(:,2)); 1];
idx =  (id1 == 0) & (id2 == 0);

contourDataOut = contourDataTemp;
contourDataOut = contourDataOut(idx == 0,:);
end