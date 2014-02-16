
    function varargout = Snakes(varargin)
    % SNAKES MATLAB code for Snakes.fig
    %      SNAKES, by itself, creates a new SNAKES or raises the existing
    %      singleton*.
    %
    %      H = SNAKES returns the handle to a new SNAKES or the handle to
    %      the existing singleton*.
    %
    %      SNAKES('CALLBACK',hObject,eventData,handles,...) calls the local
    %      function named CALLBACK in SNAKES.M with the given input arguments.
    %
    %      SNAKES('Property','Value',...) creates a new SNAKES or raises the
    %      existing singleton*.  Starting from the left, property value pairs are
    %      applied to the GUI before Snakes_OpeningFcn gets called.  An
    %      unrecognized property name or invalid value makes property application
    %      stop.  All inputs are passed to Snakes_OpeningFcn via varargin.
    %
    %      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
    %      instance to run (singleton)".
    %
    % See also: GUIDE, GUIDATA, GUIHANDLES

    % Edit the above text to modify the response to help Snakes

    % Last Modified by GUIDE v2.5 09-Nov-2013 02:40:13

    % Begin initialization code - DO NOT EDIT

    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
        'gui_Singleton',  gui_Singleton, ...
        'gui_OpeningFcn', @Snakes_OpeningFcn, ...
        'gui_OutputFcn',  @Snakes_OutputFcn, ...
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


    % --- Executes just before Snakes is made visible.
    function Snakes_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to Snakes (see VARARGIN)

    % Choose default command line output for Snakes
    global xs ys
    axes(handles.axes1)
    handles.output = hObject;
    set(handles.axes1,'buttondownfcn',@clicky);
    % Update handles structure
    guidata(hObject, handles);

    set(handles.edit1,'string','.5');
    set(handles.edit2,'string','0.20');
    set(handles.edit3,'string','.5');

    set(handles.edit5,'string','0.50');
    set(handles.edit6,'string','0.40');
    set(handles.edit7,'string','0.70');
    set(handles.edit8,'string','100');

    % UIWAIT makes Snakes wait for user response (see UIRESUME)
    % uiwait(handles.figure1);


    % --- Outputs from this function are returned to the command line.
    function varargout = Snakes_OutputFcn(hObject, eventdata, handles)
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    varargout{1} = handles.output;



    function edit1_Callback(hObject, eventdata, handles)
    % hObject    handle to edit1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of edit1 as text
    %        str2double(get(hObject,'String')) returns contents of edit1 as a double


    % --- Executes during object creation, after setting all properties.
    function edit1_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



    function edit2_Callback(hObject, eventdata, handles)
    % hObject    handle to edit2 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of edit2 as text
    %        str2double(get(hObject,'String')) returns contents of edit2 as a double


    % --- Executes during object creation, after setting all properties.
    function edit2_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit2 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



    function edit3_Callback(hObject, eventdata, handles)
    % hObject    handle to edit3 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of edit3 as text
    %        str2double(get(hObject,'String')) returns contents of edit3 as a double


    % --- Executes during object creation, after setting all properties.
    function edit3_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit3 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


    % --- Executes on key press with focus on figure1 and none of its controls.
    function figure1_KeyPressFcn(hObject, eventdata, handles)
    % hObject    handle to figure1 (see GCBO)
    % eventdata  structure with the following fields (see FIGURE)
    %	Key: name of the key that was pressed, in lower case
    %	Character: character interpretation of the key(s) that was pressed
    %	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
    % handles    structure with handles and user data (see GUIDATA)


    % --- Executes on key release with focus on figure1 and none of its controls.
    function figure1_KeyReleaseFcn(hObject, eventdata, handles)
    % hObject    handle to figure1 (see GCBO)
    % eventdata  structure with the following fields (see FIGURE)
    %	Key: name of the key that was released, in lower case
    %	Character: character interpretation of the key(s) that was released
    %	Modifier: name(s) of the modifier key(s) (i.e., control, shift) released
    % handles    structure with handles and user data (see GUIDATA)



    function edit5_Callback(hObject, eventdata, handles)
    % hObject    handle to edit5 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of edit5 as text
    %        str2double(get(hObject,'String')) returns contents of edit5 as a double


    % --- Executes during object creation, after setting all properties.
    function edit5_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit5 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



    function edit6_Callback(hObject, eventdata, handles)
    % hObject    handle to edit6 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of edit6 as text
    %        str2double(get(hObject,'String')) returns contents of edit6 as a double


    % --- Executes during object creation, after setting all properties.
    function edit6_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit6 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



    function edit7_Callback(hObject, eventdata, handles)
    % hObject    handle to edit7 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of edit7 as text
    %        str2double(get(hObject,'String')) returns contents of edit7 as a double


    % --- Executes during object creation, after setting all properties.
    function edit7_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit7 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


    % --- Executes on button press in pushbutton1.
    function pushbutton1_Callback(hObject, eventdata, handles)
    % hObject    handle to pushbutton1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    [filename, pathname] = uigetfile(...
        {'*.tif;*.jpg;*.pgm;*.gif';'*.*'},'File Selector');
    handles.filename = strcat(pathname,'\',filename);
    guidata(hObject, handles);
    handles.filename

    axes(handles.axes1)
    [x,map] = imread(handles.filename);
    axes(handles.axes1)
    imshow(x,[]);
    handles.image = x;
    handles.map=map;
    guidata(hObject, handles);
    smask = fspecial('gaussian', ceil(3*1), 1);
    y = filter2(smask, handles.image, 'same');
    %y=im2double(handles.image);
    im= imshow(y,[]);
    global xs ys
    [xs, ys] = GetSnake(handles.axes1);
    axes(handles.axes1)
    handles.imageobject = y;
    handles.xs = xs;
    handles.ys = ys;
    guidata(hObject, handles);
    %housekeeping


    % --- Executes on button press in pushbutton2.
    function pushbutton2_Callback(hObject, eventdata, handles)
    % hObject    handle to pushbutton2 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    %obtiaing various paramaeters form the text boxes
    set(hObject,'String','Busy');
    set(hObject,'BackgroundColor',[1,0,0,]);
    alpha_val = str2double(get(handles.edit1,'String'));
    beta_val = str2double(get(handles.edit2,'String'));
    gamma_val = str2double(get(handles.edit3,'String'));
    weline_val = str2double(get(handles.edit5,'String'));
    weedge_val = str2double(get(handles.edit6,'String'));
    weterm_val = str2double(get(handles.edit7,'String'));
    inter_val = str2double(get(handles.edit8,'String'));
    recursive(handles.axes1,handles.imageobject, alpha_val, beta_val, gamma_val, weline_val, weedge_val, weterm_val, inter_val);

    %minimize(handles.imageobject, handles.xs, handles.ys, alpha_val, beta_val, gamma_val, weline_val, weedge_val, weterm_val, inter_val);
    set(hObject,'String','RUN');
    set(hObject,'BackgroundColor',[0,1,0,]);
    % --- Executes when figure1 is resized.
    function figure1_ResizeFcn(hObject, eventdata, handles)
    % hObject    handle to figure1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)


    % --- Executes on mouse press over figure background, over a disabled or
    % --- inactive control, or over an axes background.
    function figure1_WindowButtonDownFcn(hObject, eventdata, handles)
    % hObject    handle to figure1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)


    % --- Executes on mouse motion over figure - except title and menu.
    function figure1_WindowButtonMotionFcn(hObject, eventdata, handles)
    % hObject    handle to figure1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)


    % --- Executes on mouse press over figure background.
    function figure1_ButtonDownFcn(hObject, eventdata, handles)
    % hObject    handle to figure1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)


    % --- Executes on key press with focus on pushbutton1 and none of its controls.
    function pushbutton1_KeyPressFcn(hObject, eventdata, handles)
    % hObject    handle to pushbutton1 (see GCBO)
    % eventdata  structure with the following fields (see UICONTROL)
    %	Key: name of the key that was pressed, in lower case
    %	Character: character interpretation of the key(s) that was pressed
    %	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
    % handles    structure with handles and user data (see GUIDATA)



    function edit8_Callback(hObject, eventdata, handles)
    % hObject    handle to edit8 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of edit8 as text
    %        str2double(get(hObject,'String')) returns contents of edit8 as a double


    % --- Executes during object creation, after setting all properties.
    function edit8_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit8 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


    % --- Executes during object creation, after setting all properties.
    function axes1_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to axes1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: place code in OpeningFcn to populate axes1


    % --- Executes during object deletion, before destroying properties.
    function axes1_DeleteFcn(hObject, eventdata, handles)
    % hObject    handle to axes1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)


    % --- Executes on mouse press over axes background.
    function axes1_ButtonDownFcn(hObject, eventdata, handles)
    % hObject    handle to axes1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)


    % --- Executes on button press in pushbutton3.
    function pushbutton3_Callback(hObject, eventdata, handles)
    % hObject    handle to pushbutton3 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    axes(handles.axes1)
    imshow(handles.image,[]);
    handles.output = hObject;
    smask = fspecial('gaussian', ceil(3*1), 1);
    y = filter2(smask, handles.image, 'same');
    handles.imageobject = y;
    % Update handles structure
    set(handles.radiobutton1,'value',1);
    set(handles.pushbutton2,'String','RUN');
    set(handles.pushbutton2,'BackgroundColor',[0,1,0,]);
    set(handles.edit1,'string','.5');
    set(handles.edit2,'string','0.20');
    set(handles.edit3,'string','.5');

    set(handles.edit5,'string','0.50');
    set(handles.edit6,'string','0.40');
    set(handles.edit7,'string','0.70');
    set(handles.edit8,'string','100');
    global xs ys
    [xs, ys] = GetSnake(handles.axes1);
    handles.imageobject = y;
    handles.xs = xs;
    handles.ys = ys;
    guidata(hObject, handles);

    % --- If Enable == 'on', executes on mouse press in 5 pixel border.
    % --- Otherwise, executes on mouse press in 5 pixel border or over radiobutton1.
    function radiobutton1_ButtonDownFcn(hObject, eventdata, handles)
    % hObject    handle to radiobutton1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)



    % --- Executes during object creation, after setting all properties.
    function uipanel6_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to uipanel6 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called


    % --- Executes during object creation, after setting all properties.
    function radiobutton1_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to radiobutton1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called


    % --- Executes when selected object is changed in uipanel6.
    function uipanel6_SelectionChangeFcn(hObject, eventdata, handles)
    % hObject    handle to the selected object in uipanel6
    % eventdata  structure with the following fields (see UIBUTTONGROUP)
    %	EventName: string 'SelectionChanged' (read only)
    %	OldValue: handle of the previously selected object or empty if none was selected
    %	NewValue: handle of the currently selected object
    % handles    structure with handles and user data (see GUIDATA)
    switch get(eventdata.NewValue,'Tag') % Get Tag of selected object.
        case 'radiobutton1'
            smask = fspecial('gaussian', ceil(3*1), 1);
            y = filter2(smask, handles.image, 'same');
            axes(handles.axes1)
            imshow(y,[]);
            handles.imageobject = y;
            guidata(hObject, handles);% Code for when radiobutton1 is selected.
        case 'radiobutton2'
            smask = fspecial('average',4);
            y = filter2(smask, handles.image, 'same');
            axes(handles.axes1)
            imshow(y,[]);
            handles.imageobject = y;
            guidata(hObject, handles);
        case 'radiobutton3'
            y=imadjust(handles.image,[],[]);
            y=double(y);
            axes(handles.axes1)
            imshow(y,[]);
            handles.imageobject = y;
            guidata(hObject, handles);
        case 'radiobutton4'
            y=adapthisteq(handles.image);
            y=double(y);
            axes(handles.axes1)
            imshow(y,[]);
            handles.imageobject = y;
            guidata(hObject, handles);
            % Code for when radiobutton2 is selected.
        case 'radiobutton5'
            y = medfilt2(handles.image,[3 3]);
            y=double(y);
            axes(handles.axes1)
            imshow(y,[]);
            handles.imageobject = y;
            guidata(hObject, handles);
        case 'togglebutton1'
            % Code for when togglebutton1 is selected.
        case 'togglebutton2'
            % Code for when togglebutton2 is selected.
            % Continue with more cases as necessary.
        otherwise
            % Code for when there is no match.
    end


    % --- Executes during object creation, after setting all properties.
    function uipanel5_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to uipanel5 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called


    function [imageobject] = recursive(axes1,image, alpha, beta, gamma, Wline, Wedge, Wterm, iterations);
    global xs ys;
    global im;
    handles.h=gcf;
    set(handles.h, 'WindowButtonUpFcn', @stopDragFcn);
    N = iterations;
    imageobject = image;
    [row col] = size(image);
    image_intensities = imageobject;
    [grady,gradx] = gradient(imageobject);
    eedge = -1 * sqrt ((gradx .* gradx + grady .* grady)); %eedge is measured by gradient in the image
    
    mask1 = [-1 1];
    mask2 = [-1;1];
    mask3 = [1 -2 1];
    mask4 = [1;-2;1];
    mask5 = [1 -1;-1 1];

    cx = conv2(imageobject,mask1,'same');
    cy = conv2(imageobject,mask2,'same');
    cxx = conv2(imageobject,mask3,'same');
    cyy = conv2(imageobject,mask4,'same');
    cxy = conv2(imageobject,mask5,'same');

    for i = 1:row
        for j= 1:col
 
            eterm(i,j) = (cyy(i,j)*cx(i,j)*cx(i,j) -2 *cxy(i,j)*cx(i,j)*cy(i,j) + cxx(i,j)*cy(i,j)*cy(i,j))/((1+cx(i,j)*cx(i,j) + cy(i,j)*cy(i,j))^1.5);
        end
    end
    Eimage = (Wline*image_intensities + Wedge*eedge -Wterm * eterm);
    [fx, fy] = gradient(Eimage);
    [m n] = size(xs);
    A = zeros(m,m);
    b = [(2*alpha + 6 *beta) -(alpha + 4*beta) beta];
    temp = zeros(1,m);
    temp(1,1:3) = temp(1,1:3) + b;
    temp(1,m-1:m) = temp(1,m-1:m) + [beta -(alpha + 4*beta)];
    for i=1:m
        A(i,:) = temp;
        temp = circshift(temp',1)';
    end
    [L U] = lu(A + gamma .* eye(m,m));
    Ainv = inv(U) * inv(L);
    for i=1:N;
        hold all;
        newx = gamma*xs - .1*interp2(fx,xs,ys);
        newy = gamma*ys - .1*interp2(fy,xs,ys);
        xs = Ainv * newx  ;
        ys = Ainv * newy ;
        im=imshow(image,[]);
        set(im,'buttondownfcn',@clicky2) %Apply spring forces by detecting mouse button down during minimization
        set(im,'HitTest','on');
        
        hold on;
        plot([xs; xs(1)], [ys; ys(1)], 'y-');
        hold off;
        pause(0.05)
    end;
    function stopDragFcn(hObject, eventdata, handles)
        handles.h=gcf;
        set(handles.h, 'WindowButtonMotionFcn', '');
        return
        
    function clicky2(hObject, eventdata, handles)
        
        global im
         handles.h=gcf;
        set(handles.h,'WindowButtonMotionFcn',@clicky1)
        %set(im,'HitTest','on');
        
    function clicky1(hObject, eventdata, handles)
    global xs ys
    global xy
    min_dist = 500;
    point = get(gca,'Currentpoint');
    x_new=point(1);
    y_new=point(3);
    len=length(xs);
    x1=[];
    y1=[];
    for i = 1:10:len
        x1=[x1,xs(i)];
        y1=[y1,ys(i)];
    end
    xy1=[x1;y1];
    len=length(xy1);
    for i = 1:len
        X=[xy1(1,i),xy1(2,i);x_new,y_new];
        distance=pdist(X,'euclidean');
        if distance<min_dist
            min_dist=distance;
            location=i;
        end
    end
    xy1(1,location)=x_new;
    xy1(2,location)=y_new;
    n=length(xy);
    t = 1:n;
    ts = 1: 0.1: n;
    xys = spline(t,xy1,ts);
    xs = xys(1,:);
    ys = xys(2,:);
    xs=xs';
    ys=ys';
    return

    function [xs, ys] = GetSnake(axes1)
    global xy;
    axis tight;
    hold on;
    [x, y] = getpts(axes1);
    x=x';y=y';
    temp=[x(1);y(1)];
    xy=[x;y];
    xy=[xy,temp];
    n=length(xy);
    t = 1:n;
    ts = 1: 0.1: n;
    xys = spline(t,xy,ts);
    xs = xys(1,:);
    ys = xys(2,:);
    xs=xs';
    ys=ys';
