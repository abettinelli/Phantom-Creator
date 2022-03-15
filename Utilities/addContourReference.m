function contourOut = addContourReference(contourOut, number, referencedUID)

contourOut.ROIs.SOPClassUID{number,1} = {referencedUID(:).SOPClassUID}';
contourOut.ROIs.SOPInstanceUID{number,1} = {referencedUID(:).SOPInstanceUID}';