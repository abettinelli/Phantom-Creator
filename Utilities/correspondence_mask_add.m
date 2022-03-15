function contourOut = correspondence_mask_add(MASKs, image_ROI, imgHeaders, contourOut)

for i =1:size(MASKs,1)
    CC_template = bwconncomp(int8(MASKs{i, 1}));
    MASKs(i, 4) = table2cell(regionprops3(CC_template,'volume'));
    MASKs(i, 5) = table2cell(regionprops3(CC_template,'PrincipalAxisLength'));
end
[~,idx] = sort([MASKs{:,4}], 'descend');
MASKs = MASKs(idx,:);
MASKs(:, 6) = {0};

CC = bwconncomp(image_ROI.Mask);
CC.MaskList = table2cell(regionprops3(CC,'Image'));
CC.PrincipalAxisLength = table2cell(regionprops3(CC,'PrincipalAxisLength'));

% Sort by descending centroid in the physical space
CC.Centroids = table2cell(regionprops3(CC,'centroid'));
temp = reshape([CC.Centroids{:}],3,[])';
[~, idx] = sort(temp(:,3),'descend');
CC.PixelIdxList = CC.PixelIdxList(1,idx);
CC.MaskList = CC.MaskList(idx,1);
CC.PrincipalAxisLength = CC.PrincipalAxisLength(idx,1);

% Sort by volume
CC.Volume = table2cell(regionprops3(CC,'Volume'));
[~, idx] = sort([CC.Volume{:}],'descend');
CC.PixelIdxList = CC.PixelIdxList(1,idx);
CC.MaskList = CC.MaskList(idx,1);
CC.PrincipalAxisLength = CC.PrincipalAxisLength(idx,1);

for n = 1:CC.NumObjects
    % Convert Binary mask to Contour Set
    curr_imagemask = zeros(size(image_ROI.Mask));
    curr_imagemask(CC.PixelIdxList{1, n}) = 1;
    tic, [contourData, referencedUID] = binaryToContourSet(imgHeaders, curr_imagemask);

    % Add the new ROI sequence to the ROIs property of dicomContours object
    [~, idx] = ismember(round(reshape([MASKs{:, 5}],3,[])',10,'significant'), round(CC.PrincipalAxisLength{n},10,'significant'), 'rows');
    idx = find(idx);
    try
    MASKs{idx, 6} = MASKs{idx, 6}+1;
    catch
        keyboard
    end
    name = [MASKs{idx, 2} '_' num2str(MASKs{idx, 6},'%02.f')];
    contourOut = addContour(contourOut,n,name,contourData,'Closed_planar',referencedUID,MASKs{idx, 3});
end