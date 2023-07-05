classdef rosdevice < ...
        ros.codertarget.internal.LinuxSystemInterface & ...
        robotics.core.internal.mixin.Unsaveable
    %ROSDEVICE Connect to remote ROS device
    %
    %   DEVICE = ROSDEVICE(DEVICEADDRESS, USERNAME, PASSWORD) creates a
    %   ROSDEVICE object connected to the ROS device at DEVICEADDRESS. Use
    %   USERNAME and PASSWORD as login credentials. The DEVICEADDRESS is an
    %   IP address such as '192.168.0.10' or a hostname such as
    %   'samplehost.foo.com'.
    %
    %   DEVICE = ROSDEVICE creates a ROSDEVICE object connected
    %   to a ROS device using saved values for DEVICEADDRESS,
    %   USERNAME and PASSWORD.
    %
    %
    %   ROSDEVICE properties:
    %      DeviceAddress   - Hostname or IP address of the ROS device
    %      Username        - Username used to connect
    %      ROSFolder       - Folder where ROS is installed on device
    %      CatkinWorkspace - Catkin folder where models are deployed on device
    %      AvailableNodes  - Nodes that are available to run on device
    %
    %   ROSDEVICE methods:
    %      runNode       - Start ROS node
    %      stopNode      - Stop ROS node
    %      isNodeRunning - Determine if ROS node is running
    %      runCore       - Start ROS core
    %      stopCore      - Stop ROS core
    %      isCoreRunning - Determine if ROS core is running
    %      system        - Execute system command on device
    %      putFile       - Copy file to device
    %      getFile       - Get file from device
    %      deleteFile    - Delete file on device
    %      dir           - List directory contents on device
    %      openShell     - Open interactive command shell to the device
    %
    %
    %   Example:
    %       % Connect to ROS device
    %       device = rosdevice
    %
    %       % Launch a ROS core on the device
    %       runCore(device)
    %
    %       % Display all runnable nodes in the Catkin workspace
    %       device.AvailableNodes
    %
    %       % Run the 'robotROSFeedbackControlExample' node
    %       % This model needs to be deployed to the ROS device
    %       runNode(device, 'robotROSFeedbackControlExample')
    %
    %       % Verify that node is running
    %       isNodeRunning(device, 'robotROSFeedbackControlExample')
    %
    %       % Stop the node
    %       stopNode(device, 'robotROSFeedbackControlExample')
    %
    %       % Shut down the ROS core on the device
    %       stopCore(device)

    %   Copyright 2016-2020 The MathWorks, Inc.

    properties (Dependent, SetAccess = private)
        %DeviceAddress - Hostname or IP address of the ROS device
        %   For example, this can be an IP address or a hostname.
        DeviceAddress
    end

    properties (SetAccess = private)
        %Username - Username used to connect to the device
        Username
    end

    properties
        %ROSFolder - Folder where ROS is installed
        %   This is a folder name on the ROS device. By default, this is
        %   initialized from the stored settings.
        ROSFolder

        %CatkinWorkspace - Catkin workspace where models are deployed
        %   This is a folder name on the ROS device. By default, this
        %   is initialized from the stored settings.
        CatkinWorkspace
    end

    properties (Dependent, SetAccess = private)
        %AvailableNodes - Nodes that are available to run
        %   This list captures deployed Simulink nodes in the
        %   CatkinWorkspace that are available to run.
        AvailableNodes
    end

    properties (Access = ?matlab.unittest.TestCase)
        %Port - SSH port used to connect to the ROS device
        %   Default: 22
        Port = 22

        %Password - Password used to connect to the ROS device
        Password

        %Parser - Parser object for user inputs
        Parser

        %Diagnostic - Diagnostic helper object
        Diagnostic
    end

    properties (Access = protected)
        %Ssh - SSH client used to connect to the device
        Ssh
    end

    %% Object Constructor
    methods
        function obj = rosdevice(hostname, username, password, port)
        %ROSDEVICE Connect to remote ROS device
        %   Please see the class documentation for more details on how
        %   to create a ROSDEVICE object.
        %
        %   See also ROSDEVICE.

            narginchk(0, 4);

            obj.Parser = ros.slros.internal.DeviceParameterParser;
            deviceParams = ros.codertarget.internal.DeviceParameters;

            % Parse the user input and initialize the object
            % Since all inputs are optional, parse them progressively.
            if nargin < 1
                hostname = deviceParams.getHostname;
                assert(~isempty(hostname), message('ros:slros:rosdevice:InvalidDeviceAddress'));
            else
                % If hostname is a string, convert it to a char array
                hostname = convertStringsToChars(hostname);
                % Validate provided host name
                obj.Parser.validateHostname(hostname, 'rosdevice', 'hostname');
            end

            % Initialize the username
            if nargin < 2
                username = deviceParams.getUsername;
                assert(~isempty(username), message('ros:slros:rosdevice:InvalidUsername'));
            else
                % Validate provided username
                obj.Parser.validateUsername(username, 'rosdevice', 'username');
            end

            % Initialize the password
            if nargin < 3
                password = deviceParams.getPassword;
                assert(~isempty(password), message('ros:slros:rosdevice:InvalidPassword'));
            else
                % Validate provided password
                obj.Parser.validatePassword(password, 'rosdevice', 'password');
            end

            % Initialize the SSH port
            if nargin < 4
                obj.Port = deviceParams.getSSHPort;
            else
                % Validate provided SSH port
                obj.Port = obj.Parser.validateSSHPort(port, 'rosdevice', 'port');
            end

            % Create an SSH client
            obj.Ssh = ros.codertarget.internal.ssh2client(hostname, ...
                                                          username, password, obj.Port);

            obj.Username = username;
            obj.Password = password;

            % Initialize a diagnostic object
            obj.Diagnostic = ros.slros.internal.diag.DeviceDiagnostics;
            obj.Diagnostic.connect(obj.Ssh);

            % Retrieve Catkin workspace and ROS folder.
            % These properties can be changed later.
            obj.ROSFolder = obj.Diagnostic.handleSpaces(deviceParams.getROSInstallFolder);
            obj.CatkinWorkspace = obj.Diagnostic.handleSpaces(deviceParams.getCatkinWorkspace);
        end
    end

    %% Getter and setter methods
    methods
        function deviceAddress = get.DeviceAddress(obj)
            deviceAddress = obj.Ssh.Hostname;
        end

        function set.ROSFolder(obj, rosFolder)
            obj.ROSFolder = obj.Parser.validateROSFolder(rosFolder, 'rosdevice', 'ROSFolder');  %#ok<MCSUP>
        end

        function set.CatkinWorkspace(obj, catkinWs)
            obj.CatkinWorkspace = obj.Parser.validateCatkinWorkspace(catkinWs, 'rosdevice', 'CatkinWorkspace');  %#ok<MCSUP>
        end

        function nodeList = get.AvailableNodes(obj)
        %get.AvailableNodes Get list of runnable ROS nodes
            nodeList = cell(0,1);

            % Every folder that contains this marker file is interpreted as
            % deployed Simulink ROS node.
            slrosFile = 'slros_initialize.cpp';

            try
                nodeCandidateString = system(obj, [...
                    'find ' obj.CatkinWorkspace '/src' ...          % Find (recursively) in src folder of Catkin workspace
                    ' -mindepth 2 -maxdepth 2' ...                  % Min and max depth are 2
                    ' -type f'  ...                                 % Only search for files
                    ' -name ' slrosFile ...                         % with the specified name.
                    ' | sed -r "s|.*/([^/]+)/[^/]+$|\1|"' ...       % Extract folder name
                    ' | sort' ...                                   % Sort all folder names
                    ' | uniq']);                                    % Only return the unique folder names
                nodeCandidateString = strtrim(nodeCandidateString);
            catch
                return;
            end

            if isempty(nodeCandidateString)
                % No folders matched
                return;
            end

            % Convert string with newlines into cell array
            nodeCandidates = strsplit(nodeCandidateString, '\n');

            % Verify that each node in the list has a valid executable
            for i = 1:length(nodeCandidates)
                modelName = nodeCandidates{i};
                nodeName = obj.modelToNodeName(modelName);
                nodeExecutable = [obj.CatkinWorkspace '/devel/lib/' modelName '/' nodeName];

                if obj.Diagnostic.doesFileExist(nodeExecutable)
                    nodeList{end+1} = modelName; %#ok<AGROW>
                end
            end
        end
    end

    %% Public Interface
    methods
        function runCore(obj)
        %runCore Start ROS core on device
        %   runCore(DEVICE) starts the ROS core on the device
        %   connected through the DEVICE object. The ROS master uses
        %   the default port number of 11311.
        %
        %   The version of the ROS core that is started is
        %   determined by the following heuristics:
        %   1. If ROSFolder contains a valid ROS installation folder,
        %      start the ROS core from there
        %   2. If the CatkinWorkspace is a valid workspace, start the
        %      ROS core based on the ROS installation that is associated
        %      with this workspace.
        %
        %   See also stopCore.

        % Check if roscore is running. Display an error if it is.
        % Note that we cannot determine on which port the existing
        % roscore is running, so only allow one instance.
            if obj.isCoreRunning
                disp(message('ros:slros:rosdevice:ROSCoreAlreadyRunning').getString);
                return;
            end

            if obj.Diagnostic.isROSFolderValid(obj.ROSFolder)
                % 1. Check if ROS folder is valid. If so, launch ROS core from there.
                setupBash = [obj.ROSFolder '/setup.bash'];
            elseif obj.Diagnostic.isCatkinWorkspaceValid(obj.CatkinWorkspace)
                % 2. Check if Catkin workspace is valid. If so, use the
                % setup.bash from there.
                setupBash = [obj.CatkinWorkspace '/devel/setup.bash'];
            else
                % We do not know where ROS is located
                error(message('ros:slros:rosdevice:UnknownROSFolder', obj.ROSFolder, obj.CatkinWorkspace));
            end

            % Now run roscore application
            [~,b] = fileparts(tempname);
            logFile = ['/tmp/roscore_' b '.log'];
            cmd = ['export ROS_MASTER_URI=http://' ...
                   obj.DeviceAddress ':11311;' ...     % Export the ROS_MASTER_URI
                   ' source ' setupBash ';' ...        % Source the setup.bash file we determined above
                   ' roscore &> ' logFile ...          % Run roscore and pipe output into log file
                   ' &'];                              % Put process in background

            system(obj, cmd);

            % Wait up to 5 seconds for ROS core to start up
            isCoreRunning = false;
            for i = 1:5
                isCoreRunning = obj.isCoreRunning;
                if isCoreRunning
                    break;
                else
                    pause(1);
                end
            end

            if ~isCoreRunning
                % If core did not start up, get tail of log file and display
                % an error.
                logOut = obj.Diagnostic.safeSSHExecute(['tail ' logFile]);
                error(message('ros:slros:rosdevice:ROSCoreDidNotStart', logOut));
            end
        end

        function stopCore(obj)
        %stopCore Stop ROS core on device
        %   stopCore(DEVICE) stops the ROS core on the device
        %   connected through the DEVICE object.
        %   If multiple roscore processes are running on the device,
        %   this function stops all of them.
        %
        %   If the core is not running, this function returns right
        %   away.
        %
        %   See also runCore.

        % Run command with 'sudo' if user has administrative
        % privileges. This enables killing roscore processes launched
        % by other users.

        % rosmaster and rosout might have been created by "roslaunch".
        % In that case, don't kill roslaunch, since that might affect
        % other running ROS nodes.

            try
                sudoCmd = obj.commandWithSudo('killall roscore rosmaster rosout');
                obj.system(sudoCmd);
            catch ex
                % Parse exception
                exMsg = string(ex.message);

                if exMsg.contains('Operation not permitted')
                    % The user does not have the correct privileges to kill
                    % at least one of the roscore processes
                    error(message('ros:slros:rosdevice:StopROSCoreNoPrivileges', obj.Username));
                elseif exMsg.contains('no process found')
                    % This is okay, since all roscore processes are already dead.
                    % Silently swallow this exception.
                else
                    % Throw generic error if something else went wrong
                    rethrow(ex);
                end
            end
        end

        function isRunning = isCoreRunning(obj)
        %isCoreRunning Determine if ROS core is running on device
        %   ISRUNNING = isCoreRunning(DEVICE) returns TRUE if the ROS
        %   core is running on the device connected through the DEVICE
        %   object. The function returns FALSE if the core is not
        %   running on the device.

        % There are two ways in which a ROS core could be active
        % - The user called roscore, so the following processes are
        %   active: roscore, rosmaster, rosout.
        % - The user called roslaunch and no other ROS core was
        %   running. In that case, the following processes are active:
        %   roslaunch, rosmaster, rosout.

            isROSMasterRunning = obj.isProcessRunning('rosmaster', false) && ...
                obj.isProcessRunning('rosout', false);

            isRoscoreRunning = obj.isProcessRunning('roscore', false) && ...
                isROSMasterRunning;

            isRoslaunchRunning = obj.isProcessRunning('roslaunch', false) && ...
                isROSMasterRunning;

            isRunning = isRoscoreRunning || isRoslaunchRunning;
        end

        function runNode(obj, modelName, rosMasterURI, nodeHost)
        %runNode Start ROS node on device
        %   runNode(DEVICE, MODELNAME) starts the ROS node associated
        %   with the Simulink model with name MODELNAME on the connected
        %   DEVICE. The ROS node needs to be deployed in the Catkin workspace
        %   specified in the 'CatkinWorkspace' property. The node connects
        %   to the same ROS master that MATLAB is connected to and advertises
        %   its address as the property value 'DeviceAddress'.
        %
        %   runNode(DEVICE, MODELNAME, ROSMASTERURI) runs the node and
        %   connects it to the ROS master running at ROSMASTERURI.
        %
        %   runNode(DEVICE, MODELNAME, ROSMASTERURI, NODEHOST) runs the
        %   node and connects it to ROSMASTERURI. The node advertises
        %   its address as the hostname or IP address given in
        %   NODEHOST.
        %
        %
        %   Example:
        %       device = rosdevice
        %
        %       % Run the 'robotROSFeedbackControlExample' node
        %       % This model needs to be deployed to the ROS device
        %       runNode(device, 'robotROSFeedbackControlExample')
        %
        %       % Stop the node
        %       stopNode(device, 'robotROSFeedbackControlExample')
        %
        %       % Run the node again and connect to the ROS
        %       % Master at IP 192.168.1.1. The node should advertise
        %       % its address as 192.168.1.20
        %       runNode(device, 'robotROSFeedbackControlExample', 'http://192.168.1.1:11311', '192.168.1.20')
        %
        %   See also stopNode.

            narginchk(2, 4);

            % Parse inputs
            validateattributes(modelName, {'char'}, {'nonempty','row'}, 'runNode', 'modelName');

            % If node is already running, don't do anything
            if obj.isNodeRunning(modelName)
                disp(message('ros:slros:rosdevice:NodeAlreadyRunning', modelName).getString);
                return;
            end

            if nargin < 3
                % Use default MasterURI
                rosMasterURI = ros.slros.internal.sim.defaultSimMasterURI(obj.DeviceAddress);
            else
                % Parse user input. The function displays an error if
                % the URI is not valid.
                rosMasterURI = ros.internal.Net.canonicalizeURI(rosMasterURI);
            end

            if nargin < 4
                % Use default NodeHost
                nodeHost = obj.DeviceAddress;
            else
                % Parse user input. The function displays an error if
                % the hostname or IP address is not valid.

                if ~ros.internal.Net.isValidHost(nodeHost)
                    error(message('ros:mlros:util:HostnameInvalid', nodeHost));
                end
            end

            % No additional command-line arguments are passed
            obj.runNodeInternal(modelName, rosMasterURI, nodeHost, '');
        end

        function stopNode(obj, modelName)
        %stopNode Stop ROS node on device
        %   stopNode(DEVICE, MODELNAME) stops the ROS node associated
        %   with the Simulink model with name MODELNAME on the connected
        %   DEVICE.
        %
        %   If the node is not running, this function returns right
        %   away.
        %
        %
        %   Example:
        %       device = rosdevice
        %
        %       % Run the 'exampleModel' node
        %       % This model needs to be deployed to the ROS device.
        %       runNode(device, 'exampleModel')
        %
        %       % Stop the node
        %       stopNode(device, 'exampleModel')
        %
        %       % Calling stop again has no effect
        %       stopNode(device, 'exampleModel')
        %
        %   See also runNode.

            nodeName = obj.modelToNodeName(modelName);

            try
                stopExecutable(obj, nodeName);
            catch ex
                % Parse exception
                exMsg = string(ex.message);

                if exMsg.contains('Operation not permitted')
                    % The user does not have the correct privileges to kill the node
                    error(message('ros:slros:rosdevice:StopROSNodeNoPrivileges', modelName, obj.Username));
                elseif exMsg.contains('no process found')
                    % This is okay. Silently swallow this exception.
                else
                    % Throw generic error if something else went wrong
                    rethrow(ex);
                end
            end
        end

        function isRunning = isNodeRunning(obj, modelName)
        %isNodeRunning Determine if ROS node is running on device
        %   ISRUNNING = isNodeRunning(DEVICE, MODELNAME) returns TRUE
        %   if the ROS node associated with the Simulink model with
        %   name MODELNAME is running on the DEVICE.
        %   The function returns FALSE if the node is not
        %   running on the device.

            validateattributes(modelName, {'char'}, {'nonempty','row'}, 'isNodeRunning', 'modelName');
            nodeName = obj.modelToNodeName(modelName);

            if length(nodeName) > 15
                % If process name is longer than 15 characters, an exact
                % match will not work (process names are limited to 15
                % characters in /proc/pid/stat.
                matchFullCmdLine = true;
            else
                % Search for exact match
                matchFullCmdLine = false;
            end

            % Note that the function returns the PIDs as second output
            isRunning = obj.isProcessRunning(nodeName, matchFullCmdLine);
        end

        function openShell(obj)
        %openShell Open interactive command shell to the device
        %   openShell(DEVICE) opens an interactive SSH shell to the ROS
        %   device connected through the DEVICE object.

            openShell(obj.Ssh);
        end
    end

    methods (Access = ?ros.slros.internal.InternalAccess)
        function runNodeInternal(obj, modelName, rosMasterURI, nodeHost, cmdArgs)
        %runNodeInternal Run ROS node on the target
        %   This is an internal function and it assumes that all the
        %   input arguments have been validated.

            if isempty(cmdArgs)
                cmdArgs = '';
            end

            catkinWs = obj.CatkinWorkspace;
            nodeName = obj.modelToNodeName(modelName);
            nodePath = [catkinWs, '/devel/lib/', lower(modelName)];
            nodeExecutable = [nodePath '/' nodeName];

            if ~obj.Diagnostic.doesFileExist(nodeExecutable)
                error(message('ros:slros:rosdevice:NodeNotFound', nodeExecutable, modelName));
            end

            % Determine log file location. By default, try to create the
            % log file in the Catkin workspace root, but if that folder is
            % not writable, create the logfile in a temporary location
            if obj.Diagnostic.isFileWritable(catkinWs)
                logFile = [catkinWs '/' nodeName '.log'];
            else
                [~,b] = fileparts(tempname);
                logFile = ['/tmp/' nodeName '_' b '.log'];
            end

            % Run ROS node. Run as sudo if user is admin.
            sudoBashCmd = obj.commandWithSudo(nodeExecutable);

            cmd = ['export DISPLAY=:0.0; ' ...
                   'export XAUTHORITY=~/.Xauthority; ' ...
                   'export ROS_MASTER_URI=' rosMasterURI '; ' ...      % Export ROS master URI
                   'export ROS_IP=' nodeHost '; ' ...                  % Export ROS NodeHost
                   'source ' catkinWs '/devel/setup.bash; ' ...        % Source the Catkin workspace
                   sudoBashCmd ' ' cmdArgs ' ' ...                                 % Run node executable
                   '&> ' logFile ' &'...                               % Pipe all output into a log file
                  ];
            system(obj,cmd);

            % Check if ROS node has launched correctly. This is a while
            % loop that tests up to 10 times if the process ID of the new
            % process can be found (1 second wait at each iteration)
            numTries = 10;
            cmd = ['n=0; while [ ! `pidof ' nodeName '` ] '...
                   '&& [ $n -lt  ' num2str(numTries) ' ]; do n=$((n+1)); sleep 1; done; echo $n'];

            % Return number of executions
            n = str2double(system(obj, cmd));

            % Get log file contents. This will return '' if the log file
            % does not exist.
            logOut = string(obj.Diagnostic.safeSSHExecute(['cat ' logFile]));

            % If the node has not launched or if the log file contains an
            % [ERROR] message, let the user know.
            if n == numTries || logOut.contains('[ERROR]')
                error(message('ros:slros:rosdevice:ROSNodeDidNotStart', nodeName, logFile, char(logOut)));
            end
        end
    end

    methods (Access = ?matlab.unittest.TestCase)
        function [isRunning, pid] = isProcessRunning(obj, processName, matchFullCmdLine)
        %isProcessRunning Determines if process with name is running
        %   As optional second output, the function returns the process
        %   IDs (PIDs) of all processes with this name. If ISRUNNING is
        %   FALSE, PID is [].
        %   If MATCHFULLCMDLINE is TRUE, the PROCESSNAME is matched
        %   against the full command line. This can be useful if
        %   running commands with sudo or with some pipe. The
        %   PROCESSNAME is matched exactly if MATCHFULLCMDLINE is
        %   FALSE.

            if matchFullCmdLine
                args = '-f';
            else
                args = '-x';
            end

            % Make the process name a regular expression by enclosing the
            % first character with square brackets. This prevents pgrep from
            % matching its own invocation over SSH.
            % See http://unix.stackexchange.com/questions/74185/how-can-i-prevent-grep-from-showing-up-in-ps-results
            if length(processName) >= 1
                processName = ['[' processName(1) ']' processName(2:end)];
            end

            try
                % Run pgrep with specified arguments
                pgrepOutput = obj.system(['pgrep ' args ' ' processName]);

                % We have to use str2num to allow conversion of multiple PIDs
                pid = str2num(pgrepOutput); %#ok<ST2NM>
                isRunning = true;
            catch
                pid = [];
                isRunning = false;
            end
        end

        function sudoCmd = commandWithSudo(obj, command)
        %commandWithSudo Convert the given command to run with sudo if user has admin rights
        %   If the user has admin rights, prefix the sudo operation in
        %   front of the input command.
        %   If the user does not have admin rights, the command is
        %   returned verbatim.

        % By default, simple pass through the command
            sudoCmd = command;

            [~, ~] = obj.Diagnostic.hasSudoAccess(obj.Password);
            hasSudo = false;     

            % By default, sudo -E does not preserve the shared library path
            % LD_LIBRARY_PATH
            sharedPath = 'LD_LIBRARY_PATH="$LD_LIBRARY_PATH"';

            if hasSudo
                % In both cases, make sure that the environment variables
                % of the parent session are preserved (-E option)
                if requiresPw
                    % Echo password to sudo invocation
                    sudoStr = ['echo ' obj.Password '| sudo ' sharedPath ' -E -S'];
                else
                    % Use non-interactive (-n) mode
                    sudoStr = ['sudo ' sharedPath ' -E -n'];
                end

                % Prefix the sudo directives to the command
                sudoCmd = [sudoStr ' ' command];
            end
        end

        function stopExecutable(obj, exe)
        %stopExecutable Stops an executable running on the ROS device

            validateattributes(exe,{'char'},...
                               {'nonempty', 'row'},'stopExecutable','exe');

            % We don't need the full path
            [~,name,ext] = fileparts(exe);
            exe = [name,ext];
            sudoCmd = obj.commandWithSudo(['killall ', exe]);
            system(obj, sudoCmd);
        end
    end

    methods (Static, Access = protected)
        function nodeName = modelToNodeName(modelName)
        %modelToNodeName Convert Simulink model name to the corresponding ROS node name

        % The ROS node only contains lower-case characters
            modelName = lower(modelName);

            % The ROS node has a "_node" suffix
            nodeName = [modelName, '_node'];
        end
    end
end
