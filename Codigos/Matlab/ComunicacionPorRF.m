function varargout = ComunicacionPorRF(varargin)
% COMUNICACIONPORRF MATLAB code for ComunicacionPorRF.fig
%      COMUNICACIONPORRF, by itself, creates a new COMUNICACIONPORRF or raises the existing
%      singleton*.
%
%      H = COMUNICACIONPORRF returns the handle to a new COMUNICACIONPORRF or the handle to
%      the existing singleton*.
%
%      COMUNICACIONPORRF('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in COMUNICACIONPORRF.M with the given input arguments.
%
%      COMUNICACIONPORRF('Property','Value',...) creates a new COMUNICACIONPORRF or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ComunicacionPorRF_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ComunicacionPorRF_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ComunicacionPorRF

% Last Modified by GUIDE v2.5 14-Jun-2018 20:54:55

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ComunicacionPorRF_OpeningFcn, ...
                   'gui_OutputFcn',  @ComunicacionPorRF_OutputFcn, ...
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


% --- Executes just before ComunicacionPorRF is made visible.
function ComunicacionPorRF_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ComunicacionPorRF (see VARARGIN)

% Choose default command line output for ComunicacionPorRF
handles.output = hObject;

handles.TXactiva=0;

handles.wA=800*ones(200,1);
handles.wB=zeros(200,1);
handles.beta=zeros(200,1);

handles.Fs=200;
% handles.tiempo=(0:1/Fs:199/Fs)';
handles.ind=(1:200)';

axes(handles.PlotBeta);
figb=plot(handles.beta);        %Guardo los datos del grafico en la variable fig
grid on;        %ploteamos al ppio y dejamos el handle
ylim([-1,3]);
ylabel('Ángulo respecto a la pista (rad)');
xlabel('Número de Muestra');
title('Ángulo respecto a la pista en Tiempo Real')

axes(handles.PlotW);
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
guidata(handles.guifig,handles);%Guarda la variable en handles
% start(handles.tmr); %inicia la cuenta del timer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
if handles.contador>10 %Solo actualizo el gráfico cada 50ms
    handles.contador=0;
    set(handles.figb,'YData',beta);%Cambio los datos del eje Y del objeto handles.fig (cambia el grafico)
    set(handles.figwA,'YData',wA);
    set(handles.figwB,'YData',wB);
end
handles.beta=beta;handles.wA=wA;handles.wB=wB;
guidata(handles.guifig,handles);%Guarda la variable en handles


% UIWAIT makes ComunicacionPorRF wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ComunicacionPorRF_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over InicioTX.
function InicioTX_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to InicioTX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in InicioTX.
function InicioTX_Callback(hObject, eventdata, handles)
% hObject    handle to InicioTX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Choose default command line output for ComunicacionPorRF
handles.output = hObject;
% handles=guidata(handles);   %carga las variables

%InicioTX el serial:
if(handles.TXactiva==0)
    %Deshabilito los botones mientras estoy activando el serial
    set(handles.InicioTX,'Enable','off');
    set(handles.AndarParar,'Enable','off');
    set(handles.RXdeDatos,'Enable','off');
    handles.TXactiva=1;
    InfoHard=instrhwinfo('serial');%Busco cuál puerto tengo conectado con esta instrucción
    handles.s=InicializacionSerial(InfoHard.SerialPorts{1},115200);
    set(handles.InicioTX,'string','Detener Comunicación');
    pause(0.1);
    %Rehabilito los botones
    set(handles.InicioTX,'Enable','on');
    set(handles.AndarParar,'Enable','on');
    set(handles.RXdeDatos,'Enable','on');
else
    stop(handles.tmr); %detengo la cuenta del timer
    set(handles.InicioTX,'string','Iniciar Comunicación');
    handles.TXactiva=0;
    set(handles.AndarParar,'string','Prender Motores');
    Env_instruccion(handles.s,'parar motores');%Le indico al nano que pare los motores
    Env_instruccion(handles.s,'stop');%Le indico al nano que pare la transmisión de datos
    fclose(instrfindall);%cierra todos los puertos activos y ocultos
    disp('Puerto Cerrado')
end
guidata(handles.guifig,handles);%Guarda la variable en handles

% --- Executes on key press with focus on InicioTX and none of its controls.
function InicioTX_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to InicioTX (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in AndarParar.
function AndarParar_Callback(hObject, eventdata, handles)
% hObject    handle to AndarParar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
texto=get(handles.AndarParar,'string');
if(texto(1)=='P' && handles.TXactiva==1)
    set(handles.AndarParar,'string','Apagar Motores');
    Env_instruccion(handles.s,'prender motores');
elseif(texto(1)=='A' && handles.TXactiva==1)
    set(handles.AndarParar,'string','Prender Motores');
    Env_instruccion(handles.s,'parar motores');
end
guidata(handles.guifig,handles);%Guarda la variable en handles

% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over AndarParar.
function AndarParar_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to AndarParar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in RXdeDatos.
function RXdeDatos_Callback(hObject, eventdata, handles)
% hObject    handle to RXdeDatos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of RXdeDatos
estado=get(hObject,'Value');
if estado==0 && handles.TXactiva==1
    Env_instruccion(handles.s,'stop');%Le indico al micro que desactive la transmisión de datos
    stop(handles.tmr); %detiene la cuenta del timer
elseif handles.TXactiva==1
    Env_instruccion(handles.s,'online');%Le indico al micro que active la transmisión de datos
    start(handles.tmr); %inicia la cuenta del timer
else
    set(hObject,'Value',0);
end
guidata(handles.guifig,handles);%Guarda la variable en handles

% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over RXdeDatos.
function RXdeDatos_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to RXdeDatos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% START USER CODE
% Necessary to provide this function to prevent timer callback
% from causing an error after GUI code stops executing.
% Before exiting, if the timer is running, stop it.
if strcmp(get(handles.tmr, 'Running'), 'on')
    stop(handles.tmr);
end
% Destroy timer
delete(handles.tmr)

if handles.TXactiva==1 %Si no está conectado al nano no le mando instrucciones
    Env_instruccion(handles.s,'parar motores');%Le indico al nano que pare los motores
    Env_instruccion(handles.s,'stop');%Le indico al nano que pare la transmisión de datos
    fclose(instrfindall);%cierra todos los puertos activos y ocultos
    disp('Puerto Cerrado')
end
% END USER CODE

% Hint: delete(hObject) closes the figure
delete(hObject);
