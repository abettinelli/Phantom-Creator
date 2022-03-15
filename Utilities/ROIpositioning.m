function varargout = ROIpositioning(varargin)
% ROIPOSITIONING MATLAB code for ROIpositioning.fig
%      ROIPOSITIONING, by itself, creates a new ROIPOSITIONING or raises the existing
%      singleton*.
%
%      H = ROIPOSITIONING returns the handle to a new ROIPOSITIONING or the handle to
%      the existing singleton*.
%
%      ROIPOSITIONING('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROIPOSITIONING.M with the given input arguments.
%
%      ROIPOSITIONING('Property','Value',...) creates a new ROIPOSITIONING or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ROIpositioning_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ROIpositioning_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ROIpositioning

% Last Modified by GUIDE v2.5 14-Mar-2022 14:25:39

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @ROIpositioning_OpeningFcn, ...
    'gui_OutputFcn',  @ROIpositioning_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before ROIpositioning is made visible.
function ROIpositioning_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ROIpositioning (see VARARGIN)

% Choose default command line output for ROIpositioning
handles.output = hObject;
handles.data = varargin{1};

maxNumberOfImages = size(handles.data.Volume,3);
h = findobj('Tag','slider1');
set(h,'Min',1)
set(h,'Max',maxNumberOfImages)
set(h,'Value',round((maxNumberOfImages+1)/2))
set(h,'SliderStep', [1/(maxNumberOfImages-1) , 10/(maxNumberOfImages-1)]);

% Image
ax1 = findobj('Tag','axes1');
set(handles.figure1,'CurrentAxes',ax1);
imagesc(handles.data.Volume(:,:,round((maxNumberOfImages+1)/2)),[-500 500])
colormap(gca, 'gray')
set(ax1,'DataAspectRatio',1./[handles.data.XPixDim handles.data.YPixDim handles.data.ZPixDim])
red = cat(3, ones(handles.data.YDim, handles.data.XDim), zeros(handles.data.YDim, handles.data.XDim), zeros(handles.data.YDim, handles.data.XDim));
hold on
%Mask
m = imshow(red);
%Lines
l1 = line([0 0], [1 handles.data.YDim],'Color','y','LineStyle','-','LineWidth', 0.25);
l2 = line([1 handles.data.XDim], [0 0],'Color','y','LineStyle','-','LineWidth', 0.25);
set(m, 'AlphaData', 0.75*handles.data.Mask(:,:,round((maxNumberOfImages+1)/2)));
% Grid
step = str2double(handles.step.String);
radius = str2double(handles.radius.String);
centre = round(([handles.data.XDim handles.data.YDim])/2);
handles.gridX = centre(1)+(0:radius-1)*step-mean((0:radius-1)*step);
handles.gridY = centre(2)+(0:radius-1)*step-mean((0:radius-1)*step);
[Y,X] = meshgrid(handles.gridX,handles.gridY);
handles.grid = [X(:) Y(:)];
plot(handles.grid(:,1), handles.grid(:,2),'g.','MarkerSize',6)
hold off
set(gca,'TickDir','in')

% TEMPLATE ROI
handles.ROIs = get_default_mask(handles);
handles.ROItemplate = handles.ROIs{1,1};
handles.nROI = 1;
h = findobj('Tag','popupmenu1');
set(h, 'String', {handles.ROIs{:,2}}');
set(h, 'Value', 1);

x = handles.data.XPixDim*size(handles.ROItemplate,2);
y = handles.data.YPixDim*size(handles.ROItemplate,1);
z = handles.data.ZPixDim*size(handles.ROItemplate,3);
max_value = max(max(x,y),z);
delta_x = floor((max_value-x)/2/handles.data.XPixDim);
delta_y = floor((max_value-y)/2/handles.data.YPixDim);
delta_z = floor((max_value-z)/2/handles.data.ZPixDim);

ax2 = findobj('Tag','axes2');
set(handles.figure1,'CurrentAxes',ax2);
imagesc((1-squeeze(handles.ROItemplate(:,:,round(size(handles.ROItemplate,3)/2))))/0.5,[0 1])
set(gca,'XTick',[]);
set(gca,'YTick',[]);
colormap(gca, 'hot')
set(gca,'XLim',[-delta_x size(handles.ROItemplate,2)+delta_x])
set(gca,'YLim',[-delta_y size(handles.ROItemplate,1)+delta_y])
set(ax2,'DataAspectRatio',1./[handles.data.YPixDim handles.data.XPixDim handles.data.ZPixDim])
ylabel('Trasversal')

ax3 = findobj('Tag','axes3');
set(handles.figure1,'CurrentAxes',ax3);
imagesc(permute((1-squeeze(handles.ROItemplate(:,round(size(handles.ROItemplate,2)/2),:)))/0.5,[2,1]),[0 1])
set(gca,'XTick',[]);
set(gca,'YTick',[]);
colormap(gca, 'hot')
set(gca,'XLim',[-delta_x size(handles.ROItemplate,2)+delta_x])
set(gca,'YLim',[-delta_z size(handles.ROItemplate,3)+delta_z])
% set(ax3,'DataAspectRatio',1./[handles.data.XPixDim handles.data.ZPixDim handles.data.YPixDim])
% zoom(handles.data.ZPixDim/handles.data.XPixDim)
ylabel('Saggital')

ax4 = findobj('Tag','axes4');
set(handles.figure1,'CurrentAxes',ax4);
imagesc(permute((1-squeeze(handles.ROItemplate(round(size(handles.ROItemplate,1)/2),:,:)))/0.5,[2,1]),[0 1])
set(gca,'XTick',[]);
set(gca,'YTick',[]);
colormap(gca, 'hot')
set(gca,'XLim',[-delta_y size(handles.ROItemplate,1)+delta_y])
set(gca,'YLim',[-delta_z size(handles.ROItemplate,3)+delta_z])
% set(ax4,'DataAspectRatio',1./[handles.data.YPixDim handles.data.ZPixDim handles.data.XPixDim])
% zoom(handles.data.ZPixDim/handles.data.YPixDim)
ylabel('Coronal')

ax5 = findobj('Tag','axes5');
set(handles.figure1,'CurrentAxes',ax5);
isosurface(handles.ROItemplate,0.5)
set(ax5,'XLim', [-delta_x size(handles.ROItemplate,2)+delta_x]);
set(ax5,'YLim', [-delta_y size(handles.ROItemplate,1)+delta_y]);
set(ax5,'ZLim', [-delta_z size(handles.ROItemplate,3)+delta_z]);
xlabel('x')
ylabel('y')
zlabel('z')
camlight
lighting gouraud
view(-45,45)
set(ax5,'DataAspectRatio',1./[handles.data.YPixDim handles.data.XPixDim handles.data.ZPixDim])

listener = addlistener(h, 'Value', 'PostSet',@(~,evt) my_update(hObject, handles, evt.AffectedObject.Value));
set(gcf, 'WindowScrollWheelFcn', {@wheel,handles});
set(gcf, 'WindowButtonMotionFcn', {@mouseMove,handles});
set(gcf, 'WindowButtonDownFcn',{@mouseClick,handles})

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ROIpositioning wait for user response (see UIRESUME)
uiwait(handles.figure1);

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
guidata(hObject, handles);
uiresume()

% --- Outputs from this function are returned to the command line.
function varargout = ROIpositioning_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
handles = guidata(hObject);
varargout{1} = handles.data;
delete(hObject);

% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles = guidata(hObject);
ax1 = handles.figure1.CurrentAxes;
axes(ax1)
z_idx = round(get(hObject,'Value'));
my_update(hObject, handles, z_idx)

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function  my_update(hObject, handles, value)
handles = guidata(hObject);
handles.value = value;
ax1 = findobj('Tag','axes1');
set(handles.figure1,'CurrentAxes',ax1);
handles.axes1.Children(end).CData = handles.data.Volume(:,:,round(handles.value));
handles.axes1.Children(end-1).AlphaData = 0.75*handles.data.Mask(:,:,round(handles.value));
guidata(hObject, handles)

function wheel(hObject, callbackdata, handles)
handles = guidata(hObject);
h = findobj('Tag','slider1');
idx = get(h,'Value');
idx_min = round(get(h,'Min'));
idx_max = round(get(h,'Max'));
if callbackdata.VerticalScrollCount < 0
    idx = idx + 1;
    idx = min(round(idx),idx_max);
    C = round(get(handles.axes1, 'CurrentPoint'));
    handles.text2.String = ['(X, Y, Z) = (', num2str(C(1,1)), ', ',num2str(C(1,2)), ', ' num2str(idx) ')'];
elseif callbackdata.VerticalScrollCount > 0
    idx = idx - 1;
    idx = max(round(idx), idx_min);
    C = round(get(handles.axes1, 'CurrentPoint'));
    handles.text2.String = ['(X, Y, Z) = (', num2str(C(1,1)), ', ',num2str(C(1,2)), ', ' num2str(idx) ')'];
end
set(h,'Value',idx);
my_update(hObject, handles, idx)

function mouseMove(hObject, callbackdata, handles)
C = round(get(handles.axes1, 'CurrentPoint'));
C3D = round(get(handles.axes5, 'CurrentPoint'));
if (C(1,1)>=1 && C(1,1)<=handles.data.XDim) && (C(1,2)>=1 && C(1,2)<=handles.data.YDim)
    handles.text2.String = ['(X, Y, Z) = (', num2str(C(1,1)), ', ',num2str(C(1,2)), ', ' num2str(round(handles.slider1.Value)) ')'];
    handles.axes1.Children(end-2).XData = [C(1,1) C(1,1)];
    handles.axes1.Children(end-3).YData = [C(1,2) C(1,2)];
else
    handles.text2.String = ['(X, Y, Z) = (-, -, ' num2str(round(handles.slider1.Value)) ')'];
    handles.axes1.Children(end-2).XData = [0 0];
    handles.axes1.Children(end-3).YData = [0 0];
end
if (C3D(1,1)>=1 && C3D(1,1)<=51) && (C3D(2,2)>=1 && C3D(1,2)<=51)
    rotate3d(handles.axes5, 'on')
else
    rotate3d(handles.axes5, 'off')
end

function mouseClick(hObject, callbackdata, handles)
handles = guidata(hObject);
C = round(get(handles.axes1, 'CurrentPoint'));
b = get(gcf,'selectiontype');

distances = sqrt(sum(bsxfun(@minus, C(1,1:2), handles.grid).^2,2));
Cnew = handles.grid(distances==min(distances),:);
if ~isempty(Cnew)
    C = Cnew;
end

% Calculate position ROI
image_dimension = size(handles.data.Mask);
roi_half_dimension = (size(handles.ROItemplate)-1)./2;
% CHECK ROI index > 1 & < dim

% ROI START-STOP INDEX
if C(1,1) <= roi_half_dimension(2)
    x_start_ROI = roi_half_dimension(2) -C(1,1) +2;
else
    x_start_ROI = 1;
end
if C(1,2) <= roi_half_dimension(1)
    y_start_ROI = roi_half_dimension(1) -C(1,2) +2;
else
    y_start_ROI = 1;
end
if round(handles.slider1.Value) <= roi_half_dimension(3)
    z_start_ROI = roi_half_dimension(3) -round(handles.slider1.Value) +2;
else
    z_start_ROI = 1;
end

if C(1,1) > image_dimension(2) -roi_half_dimension(2)
    x_end_ROI = (image_dimension(2) -C(1,1)) + (roi_half_dimension(2)+1) ;
else
    x_end_ROI = size(handles.ROItemplate,2);
end
if C(1,2) > image_dimension(1) -roi_half_dimension(1)
    y_end_ROI = (image_dimension(1) -C(1,2)) + (roi_half_dimension(1)+1);
else
    y_end_ROI = size(handles.ROItemplate,1);
end
if round(handles.slider1.Value) > image_dimension(3) -roi_half_dimension(3)
    z_end_ROI =  (image_dimension(3) -round(handles.slider1.Value)) + (roi_half_dimension(3) +1);
else
    z_end_ROI = size(handles.ROItemplate,3);
end

% IMAGE START-STOP INDEX
x_start = max(C(1,1)-roi_half_dimension(2),1);
y_start = max(C(1,2)-roi_half_dimension(1),1);
z_start = max(round(handles.slider1.Value)-roi_half_dimension(3),1);
x_end = x_start+x_end_ROI-x_start_ROI;
y_end = y_start+y_end_ROI-y_start_ROI;
z_end = z_start+z_end_ROI-z_start_ROI;

if (C(1,1)>=1 && C(1,1)<=handles.data.XDim) && (C(1,2)>=1 && C(1,2)<=handles.data.YDim)
    if strcmpi(b,'normal')
        % Update Mask
        handles.data.Mask(y_start:y_end, x_start:x_end, z_start:z_end) = (handles.data.Mask(y_start:y_end, x_start:x_end, z_start:z_end)+handles.ROItemplate(y_start_ROI:y_end_ROI,x_start_ROI:x_end_ROI,z_start_ROI:z_end_ROI))>0;
    elseif strcmpi(b,'alt')
        handles.data.Mask(y_start:y_end, x_start:x_end, z_start:z_end) = (handles.data.Mask(y_start:y_end, x_start:x_end, z_start:z_end)-handles.ROItemplate(y_start_ROI:y_end_ROI,x_start_ROI:x_end_ROI,z_start_ROI:z_end_ROI))>0;        %         selected = bwselect3(handles.data.Mask,C(1,1),C(1,2),round(handles.slider1.Value));
        %         handles.data.Mask = handles.data.Mask-selected;
    end
end

guidata(hObject, handles)
my_update(hObject, handles, round(handles.slider1.Value))

% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1
handles = guidata(hObject);
idx = get(hObject,'Value');
handles.ROItemplate = handles.ROIs{idx,1};
handles.nROI = idx;

my_update_preview(handles)
my_update(hObject, handles, round(handles.slider1.Value))

guidata(hObject, handles);

function  my_update_preview(handles)

x = handles.data.XPixDim*size(handles.ROItemplate,2);
y = handles.data.YPixDim*size(handles.ROItemplate,1);
z = handles.data.ZPixDim*size(handles.ROItemplate,3);
max_value = max(max(x,y),z);
delta_x = floor((max_value-x)/2/handles.data.XPixDim);
delta_y = floor((max_value-y)/2/handles.data.YPixDim);
delta_z = floor((max_value-z)/2/handles.data.ZPixDim);

handles.axes2.Children.CData = (1-squeeze(handles.ROItemplate(:,:,round(size(handles.ROItemplate,3)/2))))/0.5;
handles.axes2.XLim = [-delta_x size(handles.ROItemplate,2)+delta_x];
handles.axes2.YLim = [-delta_y size(handles.ROItemplate,1)+delta_y];
handles.axes2.DataAspectRatio = 1./[handles.data.YPixDim handles.data.XPixDim handles.data.ZPixDim];

handles.axes3.Children.CData = permute((1-squeeze(handles.ROItemplate(:,round(size(handles.ROItemplate,2)/2),:)))/0.5,[2 1]);
handles.axes3.XLim = [-delta_x size(handles.ROItemplate,2)+delta_x];
handles.axes3.YLim = [-delta_z size(handles.ROItemplate,3)+delta_z];
handles.axes3.DataAspectRatio = 1./[handles.data.XPixDim handles.data.ZPixDim handles.data.YPixDim];

handles.axes4.Children.CData = permute((1-squeeze(handles.ROItemplate(round(size(handles.ROItemplate,1)/2),:,:)))/0.5,[2 1]);
handles.axes4.XLim = [-delta_y size(handles.ROItemplate,1)+delta_y];
handles.axes4.YLim = [-delta_z size(handles.ROItemplate,3)+delta_z];
handles.axes4.DataAspectRatio = 1./[handles.data.YPixDim handles.data.ZPixDim handles.data.XPixDim];

set(handles.figure1,'CurrentAxes',findobj('Tag','axes5'));
cla
isosurface(handles.ROItemplate,0.5)
% set(gca,'DataAspectRatio',[1 1 1])
xlim([1 51])
ylim([1 51])
zlim([1 51])
xlabel('x')
ylabel('y')
zlabel('z')
camlight
lighting gouraud
view(-45,45)
handles.axes5.XLim = [-delta_x size(handles.ROItemplate,2)+delta_x];
handles.axes5.YLim = [-delta_y size(handles.ROItemplate,1)+delta_y];
handles.axes5.ZLim = [-delta_z size(handles.ROItemplate,3)+delta_z];
set(gca,'DataAspectRatio',1./[handles.data.YPixDim handles.data.XPixDim handles.data.ZPixDim])

% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ROIs = get_default_mask(handles)
% Case 1
ROI_temp = sphere_strel(4, handles);
ROIs{1,1} = ROIpad(ROI_temp,[29,29,29]);
ROIs{1,2}='Sphere_Small';
ROIs{1,3}=[255 0 0];

% Case 2
ROI_temp = sphere_strel(8, handles);
ROIs{2,1} = ROIpad(ROI_temp,[29,29,29]);
ROIs{2,2}='Sphere_Medium';
ROIs{2,3}=[0 255 0];

% Case 3
ROI_temp = sphere_strel(12, handles);
ROIs{3,1} = ROIpad(ROI_temp,[29,29,29]);
ROIs{3,2}='Sphere_Large';
ROIs{3,3}=[0 0 255];

mask_templates = ROIs;
save(fullfile('.','Utilities','Temp','mask_templates_gui.mat'),'mask_templates')
    
function ROImaskPadded = ROIpad(ROImask,XYZdim)

XYZroi = size(ROImask);
delta_start = ((XYZdim-1)/2+1) - round((XYZroi-1)/2);
delta_end = delta_start+XYZroi-1;
ROImaskPadded = zeros(XYZdim);
ROImaskPadded(delta_start(1):delta_end(1), delta_start(2):delta_end(2), delta_start(3):delta_end(3)) = ROImask;

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = guidata(hObject);
if handles.data.YPixDim == handles.data.XPixDim
    handles.ROItemplate = rotate3Darray(handles.ROItemplate,3);
else
    disp('Not allowed. Unisotropic voxels')
end
guidata(hObject, handles)
my_update_preview(handles)

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = guidata(hObject);
if handles.data.YPixDim == handles.data.ZPixDim
    handles.ROItemplate = rotate3Darray(handles.ROItemplate,2);
else
    disp('Not allowed. Unisotropic voxels')
end
guidata(hObject, handles)
my_update_preview(handles)

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = guidata(hObject);
if handles.data.XPixDim == handles.data.ZPixDim
    handles.ROItemplate = rotate3Darray(handles.ROItemplate,1);
else
    disp('Not allowed. Unisotropic voxels')
end
guidata(hObject, handles)
my_update_preview(handles)

function radius_Callback(hObject, eventdata, handles)
% hObject    handle to radius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of radius as text
%        str2double(get(hObject,'String')) returns contents of radius as a double
handles = guidata(hObject);
step = str2double(handles.step.String);
radius = str2double(get(hObject,'String'));
centre = round(([handles.data.XDim handles.data.YDim])/2);
handles.gridX = centre(1)+(0:radius-1)*step-mean((0:radius-1)*step);
handles.gridY = centre(2)+(0:radius-1)*step-mean((0:radius-1)*step);
[Y,X] = meshgrid(handles.gridX,handles.gridY);
handles.grid = [X(:) Y(:)];
handles.axes1.Children(end-4).XData = X(:);
handles.axes1.Children(end-4).YData = Y(:);
guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function radius_CreateFcn(hObject, eventdata, handles)
% hObject    handle to radius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function step_Callback(hObject, eventdata, handles)
% hObject    handle to step (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of step as text
%        str2double(get(hObject,'String')) returns contents of step as a double
handles = guidata(hObject);
step = str2double(get(hObject,'String'));
radius = str2double(handles.radius.String);
centre = round(([handles.data.XDim handles.data.YDim])/2);
handles.gridX = centre(1)+(0:radius-1)*step-mean((0:radius-1)*step);
handles.gridY = centre(2)+(0:radius-1)*step-mean((0:radius-1)*step);
[Y,X] = meshgrid(handles.gridX,handles.gridY);
handles.grid = [X(:) Y(:)];
handles.axes1.Children(end-4).XData = X(:);
handles.axes1.Children(end-4).YData = Y(:);
guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function step_CreateFcn(hObject, eventdata, handles)
% hObject    handle to step (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function mat_output = rotate3Darray(mat_input, dim)

switch dim
    case 1
        mat_output = permute(zeros(size(mat_input)),[1 3 2]);
    case 2
        mat_output = permute(zeros(size(mat_input)),[3 2 1]);
    case 3
        mat_output = permute(zeros(size(mat_input)),[2 1 3]);
end
for i = 1:size(mat_input,dim)
    switch dim
        case 1
            mat_output(i,:,:) = rot90(squeeze(mat_input(i,:,:)));
        case 2
            mat_output(:,i,:) = rot90(squeeze(mat_input(:,i,:)));
        case 3
            mat_output(:,:,i) = rot90(squeeze(mat_input(:,:,i)));
    end
end

% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    [file,path] = uigetfile;
    S = load(fullfile(path,file));
    fields = fieldnames(S);
    mask_templates = S.(fields{1});
    save(fullfile('.','Utilities','Temp','mask_templates_gui.mat'),'mask_templates')
    handles.ROIs = S.(fields{1});
catch
    handles.ROIs = get_default_mask(handles);
end
handles.ROItemplate = handles.ROIs{1,1};
handles.nROI = 1;

h = findobj('Tag','popupmenu1');
set(h, 'String', {handles.ROIs{:,2}}');
set(h, 'Value', 1);

my_update_preview(handles)
my_update(hObject, handles, round(handles.slider1.Value))

guidata(hObject, handles)
