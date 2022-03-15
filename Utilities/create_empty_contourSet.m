function contourOut = create_empty_contourSet(info_rt)

refUID.SOPClassUID = '';
refUID.SOPInstanceUID = '';
contourIn = dicomContours(info_rt);

n = size(contourIn.ROIs,1);
for i=n:-1:2
    contourIn = deleteContour(contourIn,i);
end

contourIn = addContourReference(contourIn, 1, refUID);
contourOut = deleteContour(contourIn,1);