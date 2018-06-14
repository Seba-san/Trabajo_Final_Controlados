function varargout = guide1(varargin)
% GUIDE1 MATLAB code for guide1.fig
%      GUIDE1, by itself, creates a new GUIDE1 or raises the existing
%      singleton*.
%
%      H = GUIDE1 returns the handle to a new GUIDE1 or the handle to
%      the existing singleton*.
%
%      GUIDE1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUIDE1.M with the given input arguments.
%
%      GUIDE1('Property','Value',...) creates a new GUIDE1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before guide1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to guide1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help guide1

% Last Modified by GUIDE v2.5 11-Jun-2018 17:53:51

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @guide1_OpeningFcn, ...
                   'gui_OutputFcn',  @guide1_OutputFcn, ...
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


% --- Executes just before guide1 is made visible.
function guide1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to guide1 (see VARARGIN)

% Choose default command line output for guide1
handles.output = hObject;

InfoHard=instrhwinfo('serial');%Busco cuál puerto tengo conectado con esta instrucción
%En InfoHard.SerialPorts me guarda celdas con los puertos disponibles. Uso
%la primer celda y chau.
handles.s=InicializacionSerial(InfoHard.SerialPorts{1},115200);

Env_instruccion(handles.s,'prender motores');
Env_instruccion(handles.s,'online');%Le indico al nano que se ponga a escupir datos sin identificador de trama

% handles.s.BytesAvailableFcnMode = 'terminator';
% handles.s.BytesAvailableFcn = @TramaDisponible;
% handles.s.Terminator='CR';%Configuro que el caracter de finalización sea el CR
% fopen(handles.s);

%EscribirSerial(handles.s,[254 0]);%Le indico al micro que desactive el controlador
set(handles.setwd,'string','Set PWM');

handles.actualizar=1;

handles.wA=800*ones(200,1);
handles.wB=zeros(200,1);
handles.beta=zeros(200,1);

handles.Fs=200;
% handles.tiempo=(0:1/Fs:199/Fs)';
handles.ind=(1:200)';

axes(handles.axes1);
figb=plot(handles.beta);        %Guardo los datos del grafico en la variable fig
grid on;        %ploteamos al ppio y dejamos el handle
ylim([-1,3]);
ylabel('Ángulo respecto a la pista (rad)');
xlabel('Número de Muestra');
title('Ángulo respecto a la pista en Tiempo Real')

axes(handles.axes2);
figwA=plot(handles.ind,handles.wA);%Guardo los datos del grafico en la variable fig
hold on;
figwB=plot(handles.ind,handles.wB);
hold off;
grid on;        %ploteamos al ppio y dejamos el handle
% ylim([0,900]);
ylabel('Velocidad Angular (rpm)');
xlabel('Número de Muestra');
title('Velocidad Angular en Tiempo Real')
legend('wA','wB')

handles.contador=0;

handles.figb=figb;
handles.figwA=figwA;
handles.figwB=figwB;
% handles.figu=figu;
guidata(hObject, handles);  %Guarda las variables en handles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Esta es la parte del timer (más la última instrucción que lo inicializa):
% START USER CODE
% Create a timer object to fire at 1/10 sec intervals
% Specify function handles for its start and run callbacks
handles.guifig=hObject; %No se para que hace esta variable
try 
    delete(handles.tmr)
end
handles.tmr = timer(...
    'ExecutionMode', 'fixedRate', ...       % Run timer repeatedly
    'Period', 5e-3, ...                        % Initial period is 1 sec.
    'TimerFcn', {@TmrFcn,handles.guifig}); % Specify callback function
% handles.tmr = timer('ExecutionMode', 'fixedRate','Period', 1,'TimerFcn', {@TmrFcn,handles.guifig});
guidata(handles.guifig,handles);    %Guarda la variable en handles
start(handles.tmr); %inicia la cuenta del timer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% END USER CODE

%INTENTO DE USAR INTERRUPCION POR SERIAL
% function TramaDisponible(obj,event) %No me anda :'(
% %Cambie el encabezado para ver si anda que se interrumpa por recepción en
% %vez de por timer
% handles=guidata(handles);   %carga las variables


%Funcion del TIMER (se ejecuta con el periodo del timer)
function TmrFcn(src,event,handles)
handles=guidata(handles);   %carga las variables

beta=handles.beta;
wA=handles.wA;
wB=handles.wB;

handles.contador=handles.contador+1;

datoBeta=fscanf(handles.s,'%f');%Lee el buffer. '%f' le indica que me lo convierta a float
datowA=fscanf(handles.s,'%f');
datowB=fscanf(handles.s,'%f');
if length(datoBeta)>0 %Esto lo pongo por si llegara a leer basura, que creo que
                      %ahí lo convierte en un elemento vacío (al menos así
                      %funciona str2num).
    beta(1:199)=beta(2:200);
    beta(200)=datoBeta;
end
if length(datowA)>0 && length(datowB)>0
    wA(1:199)=wA(2:200);
    wA(200)=datowA;
    wB(1:199)=wB(2:200);
    wB(200)=datowB;
end
if handles.actualizar==1 && handles.contador>10 %Solo actualizo el gráfico cada 50ms
    handles.contador=0;
    set(handles.figb,'YData',beta);%Cambio los datos del eje Y del objeto handles.fig (cambia el grafico)
    set(handles.figwA,'YData',wA);
    set(handles.figwB,'YData',wB);
end
handles.beta=beta;handles.wA=wA;handles.wB=wB;
guidata(handles.guifig,handles);


% --- Outputs from this function are returned to the command line.
function varargout = guide1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in setwd.
function setwd_Callback(hObject, eventdata, handles)
% hObject    handle to setwd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%cd('C:\Users\Tania\Dropbox\Trabajo final - Controlados\Codigos\Matlab')
wd=get(handles.wd_array,'String');
wd=str2double(wd);
% dato=entero2ascii(wd);
EscribirSerial(handles.s,[255 wd]);
guidata(hObject,handles);

function wd_array_Callback(hObject, eventdata, handles)
% hObject    handle to wd_array (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of wd_array as text
%        str2double(get(hObject,'String')) returns contents of wd_array as a double


% --- Executes during object creation, after setting all properties.
function wd_array_CreateFcn(hObject, eventdata, handles)
% hObject    handle to wd_array (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in controlador.
function controlador_Callback(hObject, eventdata, handles)
% hObject    handle to controlador (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of controlador
estado=get(hObject,'Value');
if estado==0
    EscribirSerial(handles.s,[254 0]);%Le indico al micro que desactive el controlador
    set(handles.setwd,'string','Set PWM');
    set(handles.wd_array,'string','0');
else
    EscribirSerial(handles.s,[254 1]);%Le indico al micro que active el controlador
    set(handles.setwd,'string','Set wd');
    EscribirSerial(handles.s,[255 800]);%Pongo la velocidad en 800 rpm
    set(handles.wd_array,'string','800');
end
guidata(hObject,handles);

% START USER CODE

% END USER CODE


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% START USER CODE
% Necessary to provide this function to prevent timer callback
% from causing an error after GUI code stops executing.
% Before exiting, if the timer is running, stop it.
if strcmp(get(handles.timer, 'Running'), 'on')
    stop(handles.timer);
end
% Destroy timer
delete(handles.timer)
delete(handles.tmr)

Env_instruccion(handles.s,'parar motores');%Le indico al nano que pare los motores
Env_instruccion(handles.s,'stop');%Le indico al nano que pare la transmisión de datos

fclose(instrfindall);%cierra todos los puertos activos y ocultos
disp('Puerto Cerrado')
% END USER CODE

% Hint: delete(hObject) closes the figure
delete(hObject);


% --- Executes on key press with focus on controlador and none of its controls.
function controlador_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to controlador (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on key press with focus on setwd and none of its controls.
function setwd_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to setwd (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1


% --- Executes on button press in checkbox2.
function checkbox2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox2
estado=get(hObject,'Value');
if estado==0
    handles.actualizar=1;
else
    handles.actualizar=0;
end
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function figure1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
