classdef Final_Project_GUI_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        WarehouseBotGUIUIFigure        matlab.ui.Figure
        SonarDistancePanel             matlab.ui.container.Panel
        IREdgeDetectionPanel           matlab.ui.container.Panel
        StatusPanel                    matlab.ui.container.Panel
        ExternalSensorsPanel           matlab.ui.container.Panel
        DestinationReachedLamp         matlab.ui.control.Lamp
        DestinationReachedLampLabel    matlab.ui.control.Label
        SonarDistanceEditField         matlab.ui.control.NumericEditField
        SonarDistanceEditFieldLabel    matlab.ui.control.Label
        IRADCEditField                 matlab.ui.control.NumericEditField
        IRADCEditFieldLabel            matlab.ui.control.Label
        ServoPosEditField              matlab.ui.control.NumericEditField
        ServoPosEditFieldLabel         matlab.ui.control.Label
        RobotSensorsPanel              matlab.ui.container.Panel
        PollSensorsButton              matlab.ui.control.Button
        RightCliffEditField            matlab.ui.control.NumericEditField
        RightCliffEditFieldLabel       matlab.ui.control.Label
        FrontRightCliffEditField       matlab.ui.control.NumericEditField
        FrontRightCliffEditFieldLabel  matlab.ui.control.Label
        LeftCliffEditField             matlab.ui.control.NumericEditField
        LeftCliffEditFieldLabel        matlab.ui.control.Label
        FrontLeftCliffEditField        matlab.ui.control.NumericEditField
        FrontLeftCliffEditFieldLabel   matlab.ui.control.Label
        RightWheelDropLamp             matlab.ui.control.Lamp
        RightWheelDropLampLabel        matlab.ui.control.Label
        LeftWheelDropLamp              matlab.ui.control.Lamp
        LeftWheelDropLampLabel         matlab.ui.control.Label
        RightBumperLamp                matlab.ui.control.Lamp
        RightBumperLampLabel           matlab.ui.control.Label
        LeftBumperLamp                 matlab.ui.control.Lamp
        LeftBumperLampLabel            matlab.ui.control.Label
        CybotSettingsPanel             matlab.ui.container.Panel
        DisconnectButton               matlab.ui.control.Button
        ConnectButton                  matlab.ui.control.Button
        ConnectedLamp                  matlab.ui.control.Lamp
        ConnectedLampLabel             matlab.ui.control.Label
        PortEditField                  matlab.ui.control.NumericEditField
        PortEditFieldLabel             matlab.ui.control.Label
        IPAddressEditField             matlab.ui.control.EditField
        IPAddressEditFieldLabel        matlab.ui.control.Label
        ControlPanel                   matlab.ui.container.Panel
        TurningAngleSpinner            matlab.ui.control.Spinner
        TurningAngleSpinnerLabel       matlab.ui.control.Label
        BackwardDistanceSpinner        matlab.ui.control.Spinner
        BackwardDistanceSpinnerLabel   matlab.ui.control.Label
        ForwardDistanceSpinner         matlab.ui.control.Spinner
        ForwardDistanceSpinnerLabel    matlab.ui.control.Label
        TurningVelSpinner              matlab.ui.control.Spinner
        TurningVelSpinnerLabel         matlab.ui.control.Label
        BackwardVelSpinner             matlab.ui.control.Spinner
        BackwardVelSpinnerLabel        matlab.ui.control.Label
        ForwardVelSpinner              matlab.ui.control.Spinner
        ForwardVelSpinnerLabel         matlab.ui.control.Label
        Scan90180Button                matlab.ui.control.Button
        Scan090Button                  matlab.ui.control.Button
        Scan180Button                  matlab.ui.control.Button
        LeftButton                     matlab.ui.control.Button
        RightButton                    matlab.ui.control.Button
        BackwardButton                 matlab.ui.control.Button
        ForwardButton                  matlab.ui.control.Button
    end

    
    properties (Constant)
        ROBOT_OP_STATUS         =   uint8(1)
        ROBOT_OP_FORWARD        =   uint8(2)
        ROBOT_OP_BACKWARD       =   uint8(3)
        ROBOT_OP_LEFT           =   uint8(4)
        ROBOT_OP_RIGHT          =   uint8(5)
        ROBOT_OP_SCAN           =   uint8(6)
        ROBOT_OP_SCAN_STATUS    =   uint8(7)
        ROBOT_OP_SCAN_COMPLETE  =   uint8(8)
        ROBOT_OP_DESTINATION    =   uint8(9)
    end
    
    properties (Access = public)
        Connection_Timer = 0 % CyBot Comm Check Timer.
        Cybot_Connection = 0 % Connection to the CyBot
        Cybot_Ping_Timeout uint8 = 0 % Counts the number of times the cybot ping timedout
        Cybot_Is_Connected logical % Tracks if the CyBot is connected
        
        % Default values for data fields
        Cybot_IP string = "192.168.1.1" % IP Address of Cybot
        Cybot_Port double = 288 % TCP port of Cybot
        Cybot_FWD_Velocity double = 150
        Cybot_BWD_Velocity double = 100
        Cybot_Turn_Velocity double = 100
        Cybot_FWD_Distance double = 500
        Cybot_BWD_Distance double = 100
        Cybot_Turn_Angle double = 90
        Cybot_Data
        Cybot_Plot_Angle
        Cybot_Plot_ADC
        Cybot_Plot_Sonar
        Cybot_Scan_Complete logical = false;

        % Polar Cordinate for Sensor Readings
        IR_Plot
        Sonar_Plot
    end

    methods (Access = public) 
        function Connection_Timer_Callback(app, ~, ~)
            write(app.Cybot_Connection, 'p');
            resp = read(app.Cybot_Connection,...
                app.Cybot_Connection.NumBytesAvailable, "string");
            if ~(strcmp(resp, "pong"))
                app.Cybot_Ping_Timeout = app.Cybot_Ping_Timeout + 1;
            elseif (app.Cybot_Ping_Timeout == 3)
                stop(app.Connection_Timer);
                app.Cybot_Is_Connected = false;
                app.ConnectedLamp.Enable = "off";
            else
                app.Cybot_Ping_Timeout = 0;
            end
        end
    end
    
    methods (Access = public)
        
        function Disable_All_Inputs(app)
            app.TurningAngleSpinner.Enable = "off";
            app.BackwardDistanceSpinner.Enable = "off";
            app.ForwardDistanceSpinner.Enable = "off";
            app.TurningVelSpinner.Enable = "off";
            app.BackwardVelSpinner.Enable = "off";
            app.ForwardVelSpinner.Enable = "off";
            app.Scan90180Button.Enable = "off";
            app.Scan090Button.Enable = "off";
            app.Scan180Button.Enable = "off";
            app.LeftButton.Enable = "off";
            app.RightButton.Enable = "off";
            app.BackwardButton.Enable = "off";
            app.ForwardButton.Enable = "off";
        end
        
        function Enable_All_inputs(app)
            app.TurningAngleSpinner.Enable = "on";
            app.BackwardDistanceSpinner.Enable = "on";
            app.ForwardDistanceSpinner.Enable = "on";
            app.TurningVelSpinner.Enable = "on";
            app.BackwardVelSpinner.Enable = "on";
            app.ForwardVelSpinner.Enable = "on";
            app.Scan90180Button.Enable = "on";
            app.Scan090Button.Enable = "on";
            app.Scan180Button.Enable = "on";
            app.LeftButton.Enable = "on";
            app.RightButton.Enable = "on";
            app.BackwardButton.Enable = "on";
            app.ForwardButton.Enable = "on";
        end

        function Connect_Cybot(app)
             app.Cybot_Connection = tcpclient(app.Cybot_IP,...
                app.Cybot_Port,"Timeout",3);
            app.Cybot_Is_Connected = true;
            configureCallback(app.Cybot_Connection, "byte", 11,...
                @app.Cybot_Recieve_Callback);
            app.Enable_All_inputs();
            app.ConnectButton.Enable ="off";
            app.DisconnectButton.Enable = "on";
        end
        
        function Disconnect_Cybot(app)
            clear app.Cybot_Connection;
            app.Disable_All_Inputs
            app.ConnectButton.Enable ="on";
            app.DisconnectButton.Enable = "off";
        end

        function Update_Bump_Sensors(app,data)
            if (bitand(data,hex2dec('08')))
                app.LeftBumperLamp.Enable ="on";
            else
                app.LeftBumperLamp.Enable ="off";
            end
            if (bitand(data,hex2dec('04')))
                app.RightBumperLamp.Enable = "on";
            else
                app.RightBumperLamp.Enable = "off";
            end
            if (bitand(data,hex2dec('02')))
                app.LeftWheelDropLamp.Enable = "on";
            else
                app.LeftWheelDropLamp.Enable = "off";
            end
            if (bitand(data,hex2dec('01')))
                app.RightWheelDropLamp.Enable = "on";
            else
                app.RightWheelDropLamp.Enable = "off";
            end
        end

        function Init_Plots(app)
            app.IR_Plot = polaraxes(app.IREdgeDetectionPanel);
            app.IR_Plot.ThetaLim = [0, 180];
            app.IR_Plot.ThetaAxisUnits = 'degrees';
            app.IR_Plot.RDir = 'reverse';
            app.IR_Plot.RLim = [0, 3000];
            app.IR_Plot.RLimMode = 'manual';
            app.IR_Plot.RAxisLocation = 90;
            app.IR_Plot.RAxisLocationMode = 'manual';
            hold(app.IR_Plot, "on");
    
    
            app.Sonar_Plot = polaraxes(app.SonarDistancePanel);
            app.Sonar_Plot.ThetaLim = [0, 180];
            app.Sonar_Plot.ThetaAxisUnits = 'degrees';
            app.Sonar_Plot.RLim = [-100, 250];
            app.Sonar_Plot.RLimMode = 'manual';
            app.Sonar_Plot.RAxisLocation = 90;
            app.Sonar_Plot.RAxisLocationMode = 'manual';
            hold(app.Sonar_Plot, "on");
        end

        function Cybot_Recieve_Callback(app,~,~)
            app.Cybot_Data = read(app.Cybot_Connection, 11, "uint8");
            if (app.Cybot_Data(1) == app.ROBOT_OP_STATUS)
                cliffLeftSignal = bitor(bitshift(uint16(app.Cybot_Data(4)),8),...
                    uint16(app.Cybot_Data(5)));
                cliffFrontLeftSignal = bitor(bitshift(uint16(app.Cybot_Data(6)),8),...
                    uint16(app.Cybot_Data(7)));
                cliffFrontRightSignal = bitor(bitshift(uint16(app.Cybot_Data(8)),8),...
                    uint16(app.Cybot_Data(9)));
                cliffRightSignal = bitor(bitshift(uint16(app.Cybot_Data(10)),8),...
                    uint16(app.Cybot_Data(11)));

                app.LeftCliffEditField.Value = double(cliffLeftSignal);
                app.FrontLeftCliffEditField.Value = double(cliffFrontLeftSignal);
                app.FrontRightCliffEditField.Value = double(cliffFrontRightSignal);
                app.RightCliffEditField.Value = double(cliffRightSignal);
                Update_Bump_Sensors(app, app.Cybot_Data(3));
            elseif (app.Cybot_Data(1) == app.ROBOT_OP_SCAN_STATUS)
                angle = app.Cybot_Data(3);
                app.ServoPosEditField.Value = double(angle);
                adc = bitor(bitshift(uint16(app.Cybot_Data(4)),8),...
                    uint16(app.Cybot_Data(5)));
                app.IRADCEditField.Value = double(adc);
                sonar = bitor(bitshift(uint16(app.Cybot_Data(6)),8),...
                    uint16(app.Cybot_Data(7)));
                app.SonarDistanceEditField.Value = double(sonar);
                app.Cybot_Plot_Angle = [app.Cybot_Plot_Angle; deg2rad(double(angle))];
                app.Cybot_Plot_ADC = [app.Cybot_Plot_ADC; adc];
                app.Cybot_Plot_Sonar = [app.Cybot_Plot_Sonar; sonar];
            elseif (app.Cybot_Data(1) == app.ROBOT_OP_SCAN_COMPLETE)
                app.Cybot_Scan_Complete = true;
            elseif (app.Cybot_Data(1) == app.ROBOT_OP_DESTINATION)
                app.DestinationReachedLamp.Enable = "on";
            end
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
        app.Init_Plots();
            
        % Load default values in UI elements
        app.ForwardVelSpinner.Value = app.Cybot_FWD_Velocity;
        app.ForwardDistanceSpinner.Value = app.Cybot_FWD_Distance;
        app.BackwardVelSpinner.Value = app.Cybot_BWD_Velocity;
        app.BackwardDistanceSpinner.Value = app.Cybot_BWD_Distance;
        app.TurningVelSpinner.Value = app.Cybot_Turn_Velocity;
        app.TurningAngleSpinner.Value = app.Cybot_Turn_Angle;
        app.IPAddressEditField.Value = app.Cybot_IP;
        app.PortEditField.Value = app.Cybot_Port;

        % Set default state for staus indicators
        app.ConnectedLamp.Enable = "off";
        app.LeftBumperLamp.Enable = "off";
        app.RightBumperLamp.Enable = "off";
        app.LeftWheelDropLamp.Enable = "off";
        app.RightWheelDropLamp.Enable = "off";
        app.ConnectButton.Enable ="on";
        app.DisconnectButton.Enable = "off";
        app.DestinationReachedLamp.Enable = "off";

        % Disable Inputs until Connected to CyBot
        Disable_All_Inputs(app);
            
        app.Connection_Timer = timer('Period',10,'ExecutionMode',...
            'fixedDelay','TasksToExecute', 1);
        %app.Connection_Timer.TimerFcn = @app.Connection_Timer_Callback; 
        app.ConnectedLamp.Enable = "off";
        end

        % Button pushed function: ForwardButton
        function ForwardButtonPushed(app, event)
            distance = uint16(app.ForwardDistanceSpinner.Value);
            velocity = uint16(app.ForwardVelSpinner.Value);
            d1 = uint8(bitshift(bitand(distance,hex2dec('FF00')),-8));
            d2 = uint8(bitand(distance,hex2dec('FF')));
            v1 = uint8(bitshift(bitand(velocity,hex2dec('FF00')),-8));
            v2 = uint8(bitand(velocity,hex2dec('FF')));
            payload = [app.ROBOT_OP_FORWARD, 4, d1, d2, v1, v2,...
                uint8(0), uint8(0)];
            for i = 1 : length(payload)
                write(app.Cybot_Connection, payload(i));
            end
        end

        % Button pushed function: LeftButton
        function LeftButtonPushed(app, event)
            angle = uint8(app.TurningAngleSpinner.Value);
            velocity = uint16(app.TurningVelSpinner.Value);
            v1 = uint8(bitshift(bitand(velocity,hex2dec('FF00')),-8));
            v2 = uint8(bitand(velocity,hex2dec('FF')));
            payload = [app.ROBOT_OP_LEFT, 3, angle, v1, v2,...
                uint8(0),uint8(0),uint8(0)];
            for i = 1 : length(payload)
                write(app.Cybot_Connection, payload(i))
            end
        end

        % Button pushed function: RightButton
        function RightButtonPushed(app, event)
            angle = uint8(app.TurningAngleSpinner.Value);
            velocity = uint16(app.TurningVelSpinner.Value);
            v1 = uint8(bitshift(bitand(velocity,hex2dec('FF00')),-8));
            v2 = uint8(bitand(velocity,hex2dec('FF')));
            payload = [app.ROBOT_OP_RIGHT, uint8(3), angle, v1, v2,...
                uint8(0),uint8(0),uint8(0)];
            for i = 1 : length(payload)
                write(app.Cybot_Connection, payload(i))
            end
        end

        % Button pushed function: BackwardButton
        function BackwardButtonPushed(app, event)
            distance = uint16(app.BackwardDistanceSpinner.Value);
            velocity = uint16(app.BackwardVelSpinner.Value);
            d1 = uint8(bitshift(bitand(distance,hex2dec('FF00')),-8));
            d2 = uint8(bitand(distance,hex2dec('FF')));
            v1 = uint8(bitshift(bitand(velocity,hex2dec('FF00')),-8));
            v2 = uint8(bitand(velocity,hex2dec('FF')));
            payload = [app.ROBOT_OP_BACKWARD, 4, d1, d2, v1, v2,0,0];
            for i = 1 : length(payload)
                write(app.Cybot_Connection, payload(i))
            end
        end

        % Button pushed function: ConnectButton
        function ConnectButtonPushed(app, event)
            Connect_Cybot(app)
            %start(app.Connection_Timer);
        end

        % Close request function: WarehouseBotGUIUIFigure
        function WarehouseBotGUIUIFigureCloseRequest(app, event)
            clear app.Cybot_Connection;
            delete(app)
        end

        % Value changed function: IPAddressEditField
        function IPAddressEditFieldValueChanged(app, event)
            value = app.IPAddressEditField.Value;
            app.Cybot_IP = value;
        end

        % Value changed function: PortEditField
        function PortEditFieldValueChanged(app, event)
            value = app.PortEditField.Value;
            app.Cybot_Port = value;
        end

        % Button pushed function: Scan090Button
        function Scan090ButtonPushed(app, event)
            app.Cybot_Scan_Complete = false;
            app.Cybot_Plot_Angle = [];
            app.Cybot_Plot_ADC = [];
            app.Cybot_Plot_Sonar = [];
            
            payload = [app.ROBOT_OP_SCAN, uint8(1), uint8(0), uint8(90), 0, 0, 0, 0];
            for j = 1 : length(payload)
                write(app.Cybot_Connection, payload(j))
            end

            while app.Cybot_Scan_Complete == false
                pause(0.5);
            end
            t = table(app.Cybot_Plot_Angle, app.Cybot_Plot_ADC, app.Cybot_Plot_Sonar);
            t.Properties.VariableNames(1:3) = {'Angle', 'ADC', 'Sonar'};
            hold(app.IR_Plot, "off");
            hold(app.Sonar_Plot, "off");
            delete(app.IR_Plot);
            delete(app.Sonar_Plot);
            app.Init_Plots();
            polarplot(app.IR_Plot, t, 'Angle', 'ADC');
            polarplot(app.Sonar_Plot, t, 'Angle', 'Sonar');
        end

        % Button pushed function: Scan90180Button
        function Scan90180ButtonPushed(app, event)
            app.Cybot_Scan_Complete = false;
            app.Cybot_Plot_Angle = [];
            app.Cybot_Plot_ADC = [];
            app.Cybot_Plot_Sonar = [];
            
            payload = [app.ROBOT_OP_SCAN, uint8(1), uint8(90), uint8(180), 0, 0, 0, 0];
            for j = 1 : length(payload)
                write(app.Cybot_Connection, payload(j))
            end

            while app.Cybot_Scan_Complete == false
                pause(0.5);
            end
            t = table(app.Cybot_Plot_Angle, app.Cybot_Plot_ADC, app.Cybot_Plot_Sonar);
            t.Properties.VariableNames(1:3) = {'Angle', 'ADC', 'Sonar'};
            hold(app.IR_Plot, "off");
            hold(app.Sonar_Plot, "off");
            delete(app.IR_Plot);
            delete(app.Sonar_Plot);
            app.Init_Plots();
            polarplot(app.IR_Plot, t, 'Angle', 'ADC');
            polarplot(app.Sonar_Plot, t, 'Angle', 'Sonar');
        end

        % Button pushed function: Scan180Button
        function Scan180ButtonPushed(app, event)
            app.Cybot_Scan_Complete = false;
            app.Cybot_Plot_Angle = [];
            app.Cybot_Plot_ADC = [];
            app.Cybot_Plot_Sonar = [];
            
            payload = [app.ROBOT_OP_SCAN, uint8(1), uint8(0), uint8(180), 0, 0, 0, 0];
            for j = 1 : length(payload)
                write(app.Cybot_Connection, payload(j))
            end

            while app.Cybot_Scan_Complete == false
                pause(0.5);
            end
            t = table(app.Cybot_Plot_Angle, app.Cybot_Plot_ADC, app.Cybot_Plot_Sonar);
            t.Properties.VariableNames(1:3) = {'Angle', 'ADC', 'Sonar'};
            hold(app.IR_Plot, "off");
            hold(app.Sonar_Plot, "off");
            delete(app.IR_Plot);
            delete(app.Sonar_Plot);
            app.Init_Plots();
            polarplot(app.IR_Plot, t, 'Angle', 'ADC');
            polarplot(app.Sonar_Plot, t, 'Angle', 'Sonar');
        end

        % Button pushed function: DisconnectButton
        function DisconnectButtonPushed(app, event)
            Disconnect_Cybot(app)
        end

        % Button pushed function: PollSensorsButton
        function PollSensorsButtonPushed(app, event)
            payload = [app.ROBOT_OP_STATUS, 0, 0, 0, 0, 0, 0, 0];
            for i = 1 : length(payload)
                write(app.Cybot_Connection, payload(i));
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create WarehouseBotGUIUIFigure and hide until all components are created
            app.WarehouseBotGUIUIFigure = uifigure('Visible', 'off');
            app.WarehouseBotGUIUIFigure.Position = [100 100 1200 950];
            app.WarehouseBotGUIUIFigure.Name = 'Warehouse Bot GUI';
            app.WarehouseBotGUIUIFigure.CloseRequestFcn = createCallbackFcn(app, @WarehouseBotGUIUIFigureCloseRequest, true);

            % Create ControlPanel
            app.ControlPanel = uipanel(app.WarehouseBotGUIUIFigure);
            app.ControlPanel.Title = 'Control';
            app.ControlPanel.Position = [1 -9 376 307];

            % Create ForwardButton
            app.ForwardButton = uibutton(app.ControlPanel, 'push');
            app.ForwardButton.ButtonPushedFcn = createCallbackFcn(app, @ForwardButtonPushed, true);
            app.ForwardButton.Position = [143 121 100 22];
            app.ForwardButton.Text = 'Forward';

            % Create BackwardButton
            app.BackwardButton = uibutton(app.ControlPanel, 'push');
            app.BackwardButton.ButtonPushedFcn = createCallbackFcn(app, @BackwardButtonPushed, true);
            app.BackwardButton.Position = [144 63 100 22];
            app.BackwardButton.Text = 'Backward';

            % Create RightButton
            app.RightButton = uibutton(app.ControlPanel, 'push');
            app.RightButton.ButtonPushedFcn = createCallbackFcn(app, @RightButtonPushed, true);
            app.RightButton.Position = [201 92 100 22];
            app.RightButton.Text = 'Right';

            % Create LeftButton
            app.LeftButton = uibutton(app.ControlPanel, 'push');
            app.LeftButton.ButtonPushedFcn = createCallbackFcn(app, @LeftButtonPushed, true);
            app.LeftButton.Position = [82 92 100 22];
            app.LeftButton.Text = 'Left';

            % Create Scan180Button
            app.Scan180Button = uibutton(app.ControlPanel, 'push');
            app.Scan180Button.ButtonPushedFcn = createCallbackFcn(app, @Scan180ButtonPushed, true);
            app.Scan180Button.Position = [261 19 100 23];
            app.Scan180Button.Text = 'Scan 180';

            % Create Scan090Button
            app.Scan090Button = uibutton(app.ControlPanel, 'push');
            app.Scan090Button.ButtonPushedFcn = createCallbackFcn(app, @Scan090ButtonPushed, true);
            app.Scan090Button.Position = [24 19 100 23];
            app.Scan090Button.Text = 'Scan 0-90';

            % Create Scan90180Button
            app.Scan90180Button = uibutton(app.ControlPanel, 'push');
            app.Scan90180Button.ButtonPushedFcn = createCallbackFcn(app, @Scan90180ButtonPushed, true);
            app.Scan90180Button.Position = [143 19 100 23];
            app.Scan90180Button.Text = 'Scan 90-180';

            % Create ForwardVelSpinnerLabel
            app.ForwardVelSpinnerLabel = uilabel(app.ControlPanel);
            app.ForwardVelSpinnerLabel.HorizontalAlignment = 'right';
            app.ForwardVelSpinnerLabel.Position = [24 209 72 22];
            app.ForwardVelSpinnerLabel.Text = 'Forward Vel.';

            % Create ForwardVelSpinner
            app.ForwardVelSpinner = uispinner(app.ControlPanel);
            app.ForwardVelSpinner.Step = 10;
            app.ForwardVelSpinner.Limits = [0 500];
            app.ForwardVelSpinner.Tooltip = {'Velocity in mm/s'};
            app.ForwardVelSpinner.Position = [17 230 100 22];

            % Create BackwardVelSpinnerLabel
            app.BackwardVelSpinnerLabel = uilabel(app.ControlPanel);
            app.BackwardVelSpinnerLabel.HorizontalAlignment = 'right';
            app.BackwardVelSpinnerLabel.Position = [148 206 81 22];
            app.BackwardVelSpinnerLabel.Text = 'Backward Vel.';

            % Create BackwardVelSpinner
            app.BackwardVelSpinner = uispinner(app.ControlPanel);
            app.BackwardVelSpinner.Step = 10;
            app.BackwardVelSpinner.Limits = [0 500];
            app.BackwardVelSpinner.Tooltip = {'Velocity in mm/s'};
            app.BackwardVelSpinner.Position = [139 230 100 22];

            % Create TurningVelSpinnerLabel
            app.TurningVelSpinnerLabel = uilabel(app.ControlPanel);
            app.TurningVelSpinnerLabel.HorizontalAlignment = 'right';
            app.TurningVelSpinnerLabel.Position = [276 207 69 22];
            app.TurningVelSpinnerLabel.Text = 'Turning Vel.';

            % Create TurningVelSpinner
            app.TurningVelSpinner = uispinner(app.ControlPanel);
            app.TurningVelSpinner.Step = 10;
            app.TurningVelSpinner.Limits = [0 500];
            app.TurningVelSpinner.Tooltip = {'Velocity in mm/s'};
            app.TurningVelSpinner.Position = [261 230 100 22];

            % Create ForwardDistanceSpinnerLabel
            app.ForwardDistanceSpinnerLabel = uilabel(app.ControlPanel);
            app.ForwardDistanceSpinnerLabel.HorizontalAlignment = 'right';
            app.ForwardDistanceSpinnerLabel.Position = [11 156 99 22];
            app.ForwardDistanceSpinnerLabel.Text = 'Forward Distance';

            % Create ForwardDistanceSpinner
            app.ForwardDistanceSpinner = uispinner(app.ControlPanel);
            app.ForwardDistanceSpinner.Tooltip = {'Distance in mm'};
            app.ForwardDistanceSpinner.Position = [18 177 100 22];

            % Create BackwardDistanceSpinnerLabel
            app.BackwardDistanceSpinnerLabel = uilabel(app.ControlPanel);
            app.BackwardDistanceSpinnerLabel.HorizontalAlignment = 'right';
            app.BackwardDistanceSpinnerLabel.Position = [132 156 108 22];
            app.BackwardDistanceSpinnerLabel.Text = 'Backward Distance';

            % Create BackwardDistanceSpinner
            app.BackwardDistanceSpinner = uispinner(app.ControlPanel);
            app.BackwardDistanceSpinner.Tooltip = {'Distance in mm'};
            app.BackwardDistanceSpinner.Position = [139 177 100 22];

            % Create TurningAngleSpinnerLabel
            app.TurningAngleSpinnerLabel = uilabel(app.ControlPanel);
            app.TurningAngleSpinnerLabel.HorizontalAlignment = 'right';
            app.TurningAngleSpinnerLabel.Position = [270 156 79 22];
            app.TurningAngleSpinnerLabel.Text = 'Turning Angle';

            % Create TurningAngleSpinner
            app.TurningAngleSpinner = uispinner(app.ControlPanel);
            app.TurningAngleSpinner.Tooltip = {'Distance in mm'};
            app.TurningAngleSpinner.Position = [262 177 100 22];

            % Create StatusPanel
            app.StatusPanel = uipanel(app.WarehouseBotGUIUIFigure);
            app.StatusPanel.Title = 'Status';
            app.StatusPanel.Position = [378 -9 823 307];

            % Create CybotSettingsPanel
            app.CybotSettingsPanel = uipanel(app.StatusPanel);
            app.CybotSettingsPanel.Title = 'Cybot Settings';
            app.CybotSettingsPanel.Position = [591 0 232 283];

            % Create IPAddressEditFieldLabel
            app.IPAddressEditFieldLabel = uilabel(app.CybotSettingsPanel);
            app.IPAddressEditFieldLabel.HorizontalAlignment = 'right';
            app.IPAddressEditFieldLabel.Position = [28 230 63 22];
            app.IPAddressEditFieldLabel.Text = 'IP Address';

            % Create IPAddressEditField
            app.IPAddressEditField = uieditfield(app.CybotSettingsPanel, 'text');
            app.IPAddressEditField.ValueChangedFcn = createCallbackFcn(app, @IPAddressEditFieldValueChanged, true);
            app.IPAddressEditField.HorizontalAlignment = 'right';
            app.IPAddressEditField.Position = [108 230 113 22];

            % Create PortEditFieldLabel
            app.PortEditFieldLabel = uilabel(app.CybotSettingsPanel);
            app.PortEditFieldLabel.HorizontalAlignment = 'right';
            app.PortEditFieldLabel.Position = [28 201 27 22];
            app.PortEditFieldLabel.Text = 'Port';

            % Create PortEditField
            app.PortEditField = uieditfield(app.CybotSettingsPanel, 'numeric');
            app.PortEditField.Limits = [0 49151];
            app.PortEditField.RoundFractionalValues = 'on';
            app.PortEditField.ValueDisplayFormat = '%11g';
            app.PortEditField.ValueChangedFcn = createCallbackFcn(app, @PortEditFieldValueChanged, true);
            app.PortEditField.Position = [184 201 38 22];

            % Create ConnectedLampLabel
            app.ConnectedLampLabel = uilabel(app.CybotSettingsPanel);
            app.ConnectedLampLabel.HorizontalAlignment = 'right';
            app.ConnectedLampLabel.Position = [123 159 64 22];
            app.ConnectedLampLabel.Text = 'Connected';

            % Create ConnectedLamp
            app.ConnectedLamp = uilamp(app.CybotSettingsPanel);
            app.ConnectedLamp.Position = [202 159 20 20];

            % Create ConnectButton
            app.ConnectButton = uibutton(app.CybotSettingsPanel, 'push');
            app.ConnectButton.ButtonPushedFcn = createCallbackFcn(app, @ConnectButtonPushed, true);
            app.ConnectButton.Position = [10 124 100 22];
            app.ConnectButton.Text = 'Connect';

            % Create DisconnectButton
            app.DisconnectButton = uibutton(app.CybotSettingsPanel, 'push');
            app.DisconnectButton.ButtonPushedFcn = createCallbackFcn(app, @DisconnectButtonPushed, true);
            app.DisconnectButton.Position = [127 124 100 23];
            app.DisconnectButton.Text = 'Disconnect';

            % Create RobotSensorsPanel
            app.RobotSensorsPanel = uipanel(app.StatusPanel);
            app.RobotSensorsPanel.Title = 'Robot Sensors';
            app.RobotSensorsPanel.Position = [0 0 326 283];

            % Create LeftBumperLampLabel
            app.LeftBumperLampLabel = uilabel(app.RobotSensorsPanel);
            app.LeftBumperLampLabel.HorizontalAlignment = 'right';
            app.LeftBumperLampLabel.Position = [45 66 71 22];
            app.LeftBumperLampLabel.Text = 'Left Bumper';

            % Create LeftBumperLamp
            app.LeftBumperLamp = uilamp(app.RobotSensorsPanel);
            app.LeftBumperLamp.Position = [131 66 20 20];
            app.LeftBumperLamp.Color = [1 0 0];

            % Create RightBumperLampLabel
            app.RightBumperLampLabel = uilabel(app.RobotSensorsPanel);
            app.RightBumperLampLabel.HorizontalAlignment = 'right';
            app.RightBumperLampLabel.Position = [183 66 79 22];
            app.RightBumperLampLabel.Text = 'Right Bumper';

            % Create RightBumperLamp
            app.RightBumperLamp = uilamp(app.RobotSensorsPanel);
            app.RightBumperLamp.Position = [277 66 20 20];
            app.RightBumperLamp.Color = [1 0 0];

            % Create LeftWheelDropLampLabel
            app.LeftWheelDropLampLabel = uilabel(app.RobotSensorsPanel);
            app.LeftWheelDropLampLabel.HorizontalAlignment = 'right';
            app.LeftWheelDropLampLabel.Position = [23 13 92 22];
            app.LeftWheelDropLampLabel.Text = 'Left Wheel Drop';

            % Create LeftWheelDropLamp
            app.LeftWheelDropLamp = uilamp(app.RobotSensorsPanel);
            app.LeftWheelDropLamp.Position = [130 13 20 20];
            app.LeftWheelDropLamp.Color = [1 0 0];

            % Create RightWheelDropLampLabel
            app.RightWheelDropLampLabel = uilabel(app.RobotSensorsPanel);
            app.RightWheelDropLampLabel.HorizontalAlignment = 'right';
            app.RightWheelDropLampLabel.Position = [166 12 100 22];
            app.RightWheelDropLampLabel.Text = 'Right Wheel Drop';

            % Create RightWheelDropLamp
            app.RightWheelDropLamp = uilamp(app.RobotSensorsPanel);
            app.RightWheelDropLamp.Position = [278 13 20 20];
            app.RightWheelDropLamp.Color = [1 0 0];

            % Create FrontLeftCliffEditFieldLabel
            app.FrontLeftCliffEditFieldLabel = uilabel(app.RobotSensorsPanel);
            app.FrontLeftCliffEditFieldLabel.HorizontalAlignment = 'right';
            app.FrontLeftCliffEditFieldLabel.Position = [25 209 80 22];
            app.FrontLeftCliffEditFieldLabel.Text = 'Front Left Cliff';

            % Create FrontLeftCliffEditField
            app.FrontLeftCliffEditField = uieditfield(app.RobotSensorsPanel, 'numeric');
            app.FrontLeftCliffEditField.Editable = 'off';
            app.FrontLeftCliffEditField.Position = [15 230 100 22];

            % Create LeftCliffEditFieldLabel
            app.LeftCliffEditFieldLabel = uilabel(app.RobotSensorsPanel);
            app.LeftCliffEditFieldLabel.HorizontalAlignment = 'right';
            app.LeftCliffEditFieldLabel.Position = [40 146 49 22];
            app.LeftCliffEditFieldLabel.Text = 'Left Cliff';

            % Create LeftCliffEditField
            app.LeftCliffEditField = uieditfield(app.RobotSensorsPanel, 'numeric');
            app.LeftCliffEditField.Editable = 'off';
            app.LeftCliffEditField.Position = [15 167 100 22];

            % Create FrontRightCliffEditFieldLabel
            app.FrontRightCliffEditFieldLabel = uilabel(app.RobotSensorsPanel);
            app.FrontRightCliffEditFieldLabel.HorizontalAlignment = 'right';
            app.FrontRightCliffEditFieldLabel.Position = [214 209 88 22];
            app.FrontRightCliffEditFieldLabel.Text = 'Front Right Cliff';

            % Create FrontRightCliffEditField
            app.FrontRightCliffEditField = uieditfield(app.RobotSensorsPanel, 'numeric');
            app.FrontRightCliffEditField.Editable = 'off';
            app.FrontRightCliffEditField.Position = [208 230 100 22];

            % Create RightCliffEditFieldLabel
            app.RightCliffEditFieldLabel = uilabel(app.RobotSensorsPanel);
            app.RightCliffEditFieldLabel.HorizontalAlignment = 'right';
            app.RightCliffEditFieldLabel.Position = [229 146 57 22];
            app.RightCliffEditFieldLabel.Text = 'Right Cliff';

            % Create RightCliffEditField
            app.RightCliffEditField = uieditfield(app.RobotSensorsPanel, 'numeric');
            app.RightCliffEditField.Editable = 'off';
            app.RightCliffEditField.Position = [208 167 100 22];

            % Create PollSensorsButton
            app.PollSensorsButton = uibutton(app.RobotSensorsPanel, 'push');
            app.PollSensorsButton.ButtonPushedFcn = createCallbackFcn(app, @PollSensorsButtonPushed, true);
            app.PollSensorsButton.Position = [113 112 100 22];
            app.PollSensorsButton.Text = 'Poll Sensors';

            % Create ExternalSensorsPanel
            app.ExternalSensorsPanel = uipanel(app.StatusPanel);
            app.ExternalSensorsPanel.Title = 'External Sensors';
            app.ExternalSensorsPanel.Position = [325 0 267 283];

            % Create ServoPosEditFieldLabel
            app.ServoPosEditFieldLabel = uilabel(app.ExternalSensorsPanel);
            app.ServoPosEditFieldLabel.HorizontalAlignment = 'right';
            app.ServoPosEditFieldLabel.Position = [59 230 64 22];
            app.ServoPosEditFieldLabel.Text = 'Servo Pos.';

            % Create ServoPosEditField
            app.ServoPosEditField = uieditfield(app.ExternalSensorsPanel, 'numeric');
            app.ServoPosEditField.Editable = 'off';
            app.ServoPosEditField.Position = [138 230 100 22];

            % Create IRADCEditFieldLabel
            app.IRADCEditFieldLabel = uilabel(app.ExternalSensorsPanel);
            app.IRADCEditFieldLabel.HorizontalAlignment = 'right';
            app.IRADCEditFieldLabel.Position = [78 167 45 22];
            app.IRADCEditFieldLabel.Text = 'IR ADC';

            % Create IRADCEditField
            app.IRADCEditField = uieditfield(app.ExternalSensorsPanel, 'numeric');
            app.IRADCEditField.Editable = 'off';
            app.IRADCEditField.Position = [138 167 100 22];

            % Create SonarDistanceEditFieldLabel
            app.SonarDistanceEditFieldLabel = uilabel(app.ExternalSensorsPanel);
            app.SonarDistanceEditFieldLabel.HorizontalAlignment = 'right';
            app.SonarDistanceEditFieldLabel.Position = [36 105 87 22];
            app.SonarDistanceEditFieldLabel.Text = 'Sonar Distance';

            % Create SonarDistanceEditField
            app.SonarDistanceEditField = uieditfield(app.ExternalSensorsPanel, 'numeric');
            app.SonarDistanceEditField.Editable = 'off';
            app.SonarDistanceEditField.Position = [138 105 100 22];

            % Create DestinationReachedLampLabel
            app.DestinationReachedLampLabel = uilabel(app.ExternalSensorsPanel);
            app.DestinationReachedLampLabel.HorizontalAlignment = 'center';
            app.DestinationReachedLampLabel.WordWrap = 'on';
            app.DestinationReachedLampLabel.Position = [53 34 70 30];
            app.DestinationReachedLampLabel.Text = 'Destination Reached';

            % Create DestinationReachedLamp
            app.DestinationReachedLamp = uilamp(app.ExternalSensorsPanel);
            app.DestinationReachedLamp.Position = [134 13 80 80];
            app.DestinationReachedLamp.Color = [0.0588 1 1];

            % Create IREdgeDetectionPanel
            app.IREdgeDetectionPanel = uipanel(app.WarehouseBotGUIUIFigure);
            app.IREdgeDetectionPanel.Title = 'IR Edge Detection';
            app.IREdgeDetectionPanel.Position = [2 297 600 654];

            % Create SonarDistancePanel
            app.SonarDistancePanel = uipanel(app.WarehouseBotGUIUIFigure);
            app.SonarDistancePanel.Title = 'Sonar Distance';
            app.SonarDistancePanel.Position = [601 297 600 654];

            % Show the figure after all components are created
            app.WarehouseBotGUIUIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = Final_Project_GUI_exported

            runningApp = getRunningApp(app);

            % Check for running singleton app
            if isempty(runningApp)

                % Create UIFigure and components
                createComponents(app)

                % Register the app with App Designer
                registerApp(app, app.WarehouseBotGUIUIFigure)

                % Execute the startup function
                runStartupFcn(app, @startupFcn)
            else

                % Focus the running singleton app
                figure(runningApp.WarehouseBotGUIUIFigure)

                app = runningApp;
            end

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.WarehouseBotGUIUIFigure)
        end
    end
end