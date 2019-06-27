function varargout = interfazTest(varargin)
% INTERFAZTEST MATLAB code for interfazTest.fig
%      INTERFAZTEST, by itself, creates a new INTERFAZTEST or raises the existing
%      singleton*.
%
%      H = INTERFAZTEST returns the handle to a new INTERFAZTEST or the handle to
%      the existing singleton*.
%
%      INTERFAZTEST('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INTERFAZTEST.M with the given input arguments.
%
%      INTERFAZTEST('Property','Value',...) creates a new INTERFAZTEST or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before interfazTest_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to interfazTest_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help interfazTest

% Last Modified by GUIDE v2.5 05-Jun-2019 00:44:46

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @interfazTest_OpeningFcn, ...
                   'gui_OutputFcn',  @interfazTest_OutputFcn, ...
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


% --- Executes just before interfazTest is made visible.
function interfazTest_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to interfazTest (see VARARGIN)

% Choose default command line output for interfazTest
handles.output = hObject;

panhandle = uipanel('Position', [0.01 0.01 0.75 0.98]);
paxvel = subplot(3,2,[1 3 5],'Parent', panhandle);
panvel = pan;
axis([0 90+10 0 120])
title ('Velocidad del vehículo')
ylabel('Velocidad (km/h)')
xlabel('Tiempo (seg)')
set(gca, 'NextPlot', 'replacechildren');
paxco = subplot(3,2,2,'Parent', panhandle);
panco = pan;
xlim([0 90+10])
title ('Emisiones de CO')
ylabel('CO (g)')
xlabel('Tiempo (seg)')
set(gca, 'NextPlot', 'replacechildren');
paxnox = subplot(3,2,4,'Parent', panhandle);
pannox = pan;
xlim([0 90+10])
title ('Emisiones de NO_{x}')
ylabel('NO_{x} (g)')
xlabel('Tiempo (seg)')
set(gca, 'NextPlot', 'replacechildren');
paxpm = subplot(3,2,6,'Parent', panhandle);
panpm = pan;
xlim([0 90+10])
title ('Emisiones de PM')
ylabel('PM (g)')
xlabel('Tiempo (seg)')
set(gca, 'NextPlot', 'replacechildren');

handles.graficas.panhandle = panhandle;
handles.graficas.paxvel = paxvel;
handles.graficas.panvel = panvel;
handles.graficas.paxco = paxco;
handles.graficas.panco = panco;
handles.graficas.paxnox = paxnox;
handles.graficas.pannox = pannox;
handles.graficas.paxpm = paxpm;
handles.graficas.panpm = panpm;

% Update handles structure
guidata(hObject, handles);

% axes(handles.grafica,'Position',[0.05, 0.05, 0.4, 0.9]);
% axes(handles.grafica,'Position',[0.55, 0.71, 0.4, 0.23]);
% axes(handles.grafica,'Position',[0.55, 0.38, 0.4, 0.23]);
% axes(handles.grafica,'Position',[0.55, 0.05, 0.4, 0.23]);

% UIWAIT makes interfazTest wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = interfazTest_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in listaSerial.
function listaSerial_Callback(hObject, eventdata, handles)
% hObject    handle to listaSerial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listaSerial contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listaSerial


% --- Executes during object creation, after setting all properties.
function listaSerial_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listaSerial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in botonStart.
function botonStart_Callback(hObject, eventdata, handles)
% hObject    handle to botonStart (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global vel co nox pm av_speed copert_vel latitud longitud
if strcmp(handles.botonStart.String, 'Start')
    vel = []; latitud = []; longitud = []; co = []; nox = []; pm = []; copert_vel = [];
    av_speed = zeros(1,10); 
    handles.euroStd.Visible = 'off';
    handles.listaSerial.Enable = 'inactive';
    handles.botonRefresh.Enable = 'inactive';
    handles.graficas.panvel.Enable = 'off';
    handles.graficas.panco.Enable = 'off';
    handles.graficas.pannox.Enable = 'off';
    handles.graficas.panpm.Enable = 'off';
    set(handles.graficas.paxvel, 'XLim', [0 90+10])
    set(handles.graficas.paxco, 'XLim', [0 90+10])
    set(handles.graficas.paxnox, 'XLim', [0 90+10])
    set(handles.graficas.paxpm, 'XLim', [0 90+10])
    lista = handles.listaSerial.String;
    seleccion = handles.listaSerial.Value;
    handles.serialConnection = serial(lista{seleccion});
    set(handles.serialConnection, 'BaudRate', 38400, 'Terminator', 'CR');
    handles.serialConnection.BytesAvailableFcn = @(src,event) readDataFormat(src,event,handles.graficas);
    fopen(handles.serialConnection);
    normativa = handles.euroStd.SelectedObject;
    if normativa == handles.euro3button
        fprintf(handles.serialConnection, 'euro3');
    else
        fprintf(handles.serialConnection, 'euro6');
    end
    fprintf(handles.serialConnection, 'test');
    handles.botonStart.String = 'Stop';
else
    fecha = clock;
    fOutput = fopen(['speed_' num2str(fecha(1)) '_' num2str(fecha(2)) '_' num2str(fecha(3)) '_' num2str(fecha(4)) '_' num2str(fecha(5)) '_' num2str(round(fecha(6)*1000)) '.csv'], 'w');
    fprintf(fOutput, 'speed, copert_speed\r');
    fprintf(fOutput, '%.2f, %.2f\r', [vel; copert_vel]);
    fclose(fOutput);
    fOutput = fopen(['emissions_' num2str(fecha(1)) '_' num2str(fecha(2)) '_' num2str(fecha(3)) '_' num2str(fecha(4)) '_' num2str(fecha(5)) '_' num2str(round(fecha(6)*1000)) '.csv'], 'w');
    fprintf(fOutput, 'co, nox, pm\r');
    fprintf(fOutput, '%E, %E, %E\r', [co; nox; pm]);
    fclose(fOutput);
    fOutput = fopen(['coordenadas' num2str(fecha(1)) '_' num2str(fecha(2)) '_' num2str(fecha(3)) '_' num2str(fecha(4)) '_' num2str(fecha(5)) '_' num2str(round(fecha(6)*1000)) '.csv'], 'w');
    fprintf(fOutput, 'latitud, longitud\r');
    fprintf(fOutput, '%E, %E\r', [latitud; longitud]);
    fclose(fOutput);
    handles.botonStart.String = 'Start';
    handles.euroStd.Visible = 'on';
    handles.listaSerial.Enable = 'on';
    handles.botonRefresh.Enable = 'on';
    % handles.graficas.panvel.Motion = 'horizontal';
    handles.graficas.panvel.Enable = 'on';
    % handles.graficas.panco.Motion = 'horizontal';
    handles.graficas.panco.Enable = 'on';
    % handles.graficas.pannox.Motion = 'horizontal';
    handles.graficas.pannox.Enable = 'on';
    % handles.graficas.panpm.Motion = 'horizontal';
    handles.graficas.panpm.Enable = 'on';
    fprintf(handles.serialConnection, 'test');
    fclose(handles.serialConnection);
end

guidata(hObject, handles);

% --- Executes on button press in botonRefresh.
function botonRefresh_Callback(hObject, eventdata, handles)
% hObject    handle to botonRefresh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

lista = seriallist;
for i = 1:length(lista)
    entradas{i} = lista(i);
end
handles.listaSerial.String = entradas;

function readDataFormat(hObject, eventdata, handle_to_output_to)
% hObject    handle to botonRefresh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vel co nox pm av_speed copert_vel latitud longitud
[A, count] = fscanf(hObject, '{"speed":[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d],"lat":%E,"long":%E,"co":%E,"nox":%E,"pm":%E}');
if count ~= 0
    vel((end+1):(end+10)) = A(1:10);
    latitud(end+1) = A(11);
    longitud(end+1) = A(12);
    co(end+1) = A(13);
    nox(end+1) = A(14);
    pm(end+1) = A(15);
    for k = 1:10
        av_speed(1) = vel(end-10+k);
        copert_vel(end+1) = sum(av_speed)/10;
        av_speed = circshift(av_speed,1);
    end
    if length(vel) > 90/0.5
        plot(handle_to_output_to.paxvel,.5:.5:length(vel)*.5, vel, .5:.5:length(vel)*.5, copert_vel)
        set(handle_to_output_to.paxvel, 'XLim', [(length(vel)*.5-90) (length(vel)*.5)+10], 'XLim', [(length(vel)*.5-90) (length(vel)*.5)+10])
        bar(handle_to_output_to.paxco, 5:5:length(co)*5, co)
        set(handle_to_output_to.paxco, 'XLim', [(length(vel)*.5-90) (length(vel)*.5)+10], 'YLim', [0 max(co)])
        bar(handle_to_output_to.paxnox, 5:5:length(nox)*5, nox)
        set(handle_to_output_to.paxnox, 'XLim', [(length(vel)*.5-90) (length(vel)*.5)+10], 'YLim', [0 max(nox)])
        bar(handle_to_output_to.paxpm, 5:5:length(pm)*5, pm)
        set(handle_to_output_to.paxpm, 'XLim', [(length(vel)*.5-90) (length(vel)*.5)+10], 'YLim', [0 max(pm)])
%         plot(handle_to_output_to.paxvel,(length(vel)*.5-90+.5):.5:length(vel)*.5, vel((end-90/.5+1):end)) %, copert_vel((end-90/.5+1):end))
%         set(handle_to_output_to.paxvel, 'XLim', [(length(vel)*.5-90) (length(vel)*.5)+10])
%         bar(handle_to_output_to.paxco,(length(co)*5-90+5):5:length(co)*5, co((end-90/5+1):end))
%         set(handle_to_output_to.paxvel, 'XLim', [(length(vel)*.5-90) (length(vel)*.5)+10])
%         bar(handle_to_output_to.paxnox,(length(nox)*5-90+5):5:length(nox)*5, nox((end-90/5+1):end))
%         set(handle_to_output_to.paxvel, 'XLim', [(length(vel)*.5-90) (length(vel)*.5)+10])
%         bar(handle_to_output_to.paxpm,(length(pm)*5-90+5):5:length(pm)*5, pm((end-90/5+1):end))
%         set(handle_to_output_to.paxvel, 'XLim', [(length(vel)*.5-90) (length(vel)*.5)+10])
    else
        plot(handle_to_output_to.paxvel,.5:.5:length(vel)*.5, vel, .5:.5:length(vel)*.5, copert_vel)
        bar(handle_to_output_to.paxco,5:5:length(co)*5, co)
        bar(handle_to_output_to.paxnox,5:5:length(nox)*5, nox)
        bar(handle_to_output_to.paxpm,5:5:length(pm)*5, pm)
    end
end
