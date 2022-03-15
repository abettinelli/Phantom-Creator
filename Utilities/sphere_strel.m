function SE = sphere_strel(radius_mm,  handles)

% 3D ellipsoid

x_dim = handles.data.XPixDim;
y_dim = handles.data.YPixDim;
z_dim = handles.data.ZPixDim;

x_vox_rad = radius_mm/x_dim;
y_vox_rad = radius_mm/y_dim;
z_vox_rad = radius_mm/z_dim;

%Ranges 
xbase = 1:2*ceil(x_vox_rad)+1; 
ybase = 1:2*ceil(y_vox_rad)+1; 
zbase = 1:2*ceil(z_vox_rad)+1; 
[xm,ym,zm] = ndgrid(xbase,ybase,zbase) ;

%Centers 
xc = ceil(x_vox_rad) + 1; 
yc = ceil(y_vox_rad) + 1; 
zc = ceil(z_vox_rad) + 1;

SE = logical( ((xm-xc).^2/(x_vox_rad.^2)) + ((ym-yc).^2/(y_vox_rad.^2)) + ((zm-zc).^2/(z_vox_rad.^2)) <= 1 ) ;