function varargout = gui1(varargin)
% GUI1 MATLAB code for gui1.fig
%      GUI1, by itself, creates a new GUI1 or raises the existing
%      singleton*.
%
%      H = GUI1 returns the handle to a new GUI1 or the handle to
%      the existing singleton*.
%
%      GUI1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI1.M with the given input arguments.
%
%      GUI1('Property','Value',...) creates a new GUI1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui1

% Last Modified by GUIDE v2.5 13-Apr-2024 01:30:55

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui1_OpeningFcn, ...
                   'gui_OutputFcn',  @gui1_OutputFcn, ...
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


% --- Executes just before gui1 is made visible.
function gui1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui1 (see VARARGIN)

% Choose default command line output for gui1
handles.output = hObject;
imshow('bgd.png','parent',handles.axes1);
imshow('axesimage.gif','parent',handles.axes2);
imshow('axesimage.gif','parent',handles.axes3);
imshow('axesimage.gif','parent',handles.axes4);
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes gui1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in select.
function select_Callback(hObject, eventdata, handles)
% hObject    handle to select (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global I
imshow('axesimage.gif','parent',handles.axes2);
imshow('axesimage.gif','parent',handles.axes3);
imshow('axesimage.gif','parent',handles.axes4);
[fname,path]=uigetfile({'*.png;*.jpg'});
I=imread(fullfile(path,fname));
if size(I,3)==3
    I=rgb2gray(I);
end
imshow(I,'parent',handles.axes2);
title('Input Image','parent',handles.axes2);
% --- Executes on button press in addnoise.
function addnoise_Callback(hObject, eventdata, handles)
% hObject    handle to addnoise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global noise
contents = cellstr(get(handles.noisemenu,'String'));
noise=str2double(contents{get(handles.noisemenu,'Value')});

% --- Executes on button press in coils.
function coils_Callback(hObject, eventdata, handles)
% hObject    handle to coils (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global noise I im coils
I=double(I);
contents = cellstr(get(handles.coilsmenu,'String'));
coils=str2double(contents{get(handles.coilsmenu,'Value')});
[im, params, M0, Kn, K0]=phantom_parallel(I,coils,noise ,.5,0,[2,32]);

% --- Executes on button press in display_noisy_image.
function display_noisy_image_Callback(hObject, eventdata, handles)
% hObject    handle to display_noisy_image (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global im
axes(handles.axes3);
imshow(im,[]);
title('Noisy Image','parent',handles.axes3);

% --- Executes on button press in nlml.
function nlml_Callback(hObject, eventdata, handles)
% hObject    handle to nlml (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global im noise coils OutANLML I mse_nlml mse_anlml mad_nlml mad_anlml ssim_nlml ssim_anlml psnr_nlml psnr_anlml VC1 VC2
if size(im,3)==3
    im=rgb2gray(im);
end

%size of image
[r,c]=size(im); 

s=7; %noise level

%% 1. Noise estimation With the Necessity of a Background
thresM=3*s;
mask =im2bw(1-double(imfill(im>thresM,'holes')));
    sigma2=0.5.*(sum((im(mask)).^2))./sum(mask(:));
   sigma=sqrt(sigma2);
       
%sigma=Nsigmaest2(im); 
%number of coils 
L=coils; 
%update sigma
sigma2=sigma;
%normalize
%No of non local neighboring pixels
N=12;
%Add zeros to the boundary
im2=[im,zeros(r,N)];
OutNLML=im;
OutANLML=im;
wb=waitbar(0,'please wait......');

%ML estimation
for i=2:r-1
    for j=2:c-1
        %waitbar(((i-2)*j+i)/(r*c));
        %Signal estimation using NLML method
        A=im(i-1:i+1,j-1:j+1);
        mi=A(:);
        lnLA=lnL(mi,A,L,sigma2);
        A_ml=max(max(lnLA));
         OutNLML(i,j)=A_ml;
        %signal estimation using adaptive NLML
        neigborsA=A(:);
        neigborsA(5)=[];
        D=zeros(1,N);
        for n=1:N
            R=im2(i-1:i+1,j+n-1:j+n+1);%neigborhood region
            neigborsR=R(:);
            neigborsR(5)=[];            
            D(n)=dist( neigborsA',neigborsR);
        end
         D=sort(D);
         lnLsigma=lnL(D,A,L,sigma2);
         Diffsigma=abs(lnLsigma-sigma2);
         [minvalue,k]=min(Diffsigma(:));
         OutANLML(i,j)=lnLA(k); 
    end
end
waitbar(1);
axes(handles.axes4);
imshow(abs(OutNLML),[]);title( 'NLML Output');
% figure,imshow(abs(OutANLML),[]);title( 'ANLML Output');
close(wb);
VC1=OutNLML;
VC2=OutANLML;
% normalize and map into 0-255 range
OutNLML1=uint8((OutNLML./mean(OutNLML(:)))*255);
OutANLML1=uint8((OutANLML./mean(OutANLML(:)))*255);
%1.Structural similarity index measure
OutNLML1=double(OutNLML1);
OutANLML1=double(OutANLML1);
ssim_nlml = ssim(double(I),OutNLML1);
ssim_anlml = ssim(double(I),OutANLML1);

%2.Mean Squared Error
%calculate the "square error" image.
squaredErrorImage = (double(I) - double(OutNLML1)) .^ 2;
% Sum the Squared Image and divide by the number of elements
% to get the Mean Squared Error. 
mse_nlml = sum(sum(squaredErrorImage))/numel(squaredErrorImage);

squaredErrorImage = (double(I) - double(OutANLML1)) .^ 2;
mse_anlml = sum(sum(squaredErrorImage))/numel(squaredErrorImage);

%3.Peak signal to Noise Ratio
% Calculate PSNR  from the MSE  
psnr_nlml = 20 * log10( 256^2 / mse_nlml);
psnr_anlml = 20 * log10( 256^2 / mse_anlml);
%4.Maximum Absolute Difference
error =abs( double(I) - OutNLML1);
mad_nlml = max(max(error));
error =abs( double(I) - OutANLML1);
mad_anlml = max(max(error));

% --- Executes on button press in adaptivenlml.
function adaptivenlml_Callback(hObject, eventdata, handles)
% hObject    handle to adaptivenlml (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global OutANLML
axes(handles.axes4);
imshow(abs(OutANLML),[]);title( 'ANLML Output');
% --- Executes on button press in extendedlmmse.

function noisemenu_Callback(hObject, eventdata, handles)
% hObject    handle to noisemenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns noisemenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from noisemenu


% --- Executes during object creation, after setting all properties.
function noisemenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to noisemenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in coilsmenu.
function coilsmenu_Callback(hObject, eventdata, handles)
% hObject    handle to coilsmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns coilsmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from coilsmenu


% --- Executes during object creation, after setting all properties.
function coilsmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to coilsmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in QAnalysis.
function QAnalysis_Callback(hObject, eventdata, handles)
% hObject    handle to QAnalysis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global  mse_nlml mse_anlml mad_nlml mad_anlml ssim_nlml ...
    ssim_anlml psnr_nlml psnr_anlml
fprintf('NLML-Structural similarity index measure = %f\n',ssim_nlml);
fprintf('ANLML-Structural similarity index measure = %f\n',ssim_anlml);


fprintf('NLML-Mean Squared Error = %f\n',mse_nlml);
fprintf('ANLML-Mean Squared Error = %f\n',mse_anlml);



fprintf('NLML-Peak signal to Noise Ratio = %f\n',psnr_nlml);
fprintf('ANLML-Peak signal to Noise Ratio = %f\n',psnr_anlml);


fprintf('NLML-Maximum Difference = %f\n',mad_nlml); 
fprintf('ANLML-Maximum Difference = %f\n',mad_anlml);



%--- Executes on button press in visualComparison.
function visualComparison_Callback(hObject, eventdata, handles)
% hObject    handle to visualComparison (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global VC1 VC2 
figure;
subplot(121);
imshow(VC1,[]); title('NLML')
subplot(122);
imshow(VC2,[]); title('Adaptive NLML')
