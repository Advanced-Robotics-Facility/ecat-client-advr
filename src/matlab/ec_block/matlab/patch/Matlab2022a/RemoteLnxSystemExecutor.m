classdef RemoteLnxSystemExecutor < ros.codertarget.internal.SystemInterface & ...
        ros.codertarget.internal.LinuxSystemInterface & ...
        robotics.core.internal.mixin.Unsaveable
    %REMOTELNXSYSTEMEXECUTOR Implements SystemInterface for a remote Linux
    %device
    
    %   Copyright 2021 The MathWorks, Inc.
    properties (Access = protected)
        Ssh
    end
    
    properties (Hidden, Access = {?ros.codertarget.internal.RemoteLnxCoreExecutor,...
            ?ros.codertarget.internal.LnxNodeExecutor})
        SudoPassword
        DeviceAddress
    end
    
    methods
        function obj = RemoteLnxSystemExecutor(deviceAddress,varargin)
            obj.Ssh = ros.codertarget.internal.ssh2client(deviceAddress,varargin{:});
            obj.DeviceAddress = deviceAddress;
            obj.SudoPassword = varargin{2};
        end
        
        function sshObj = getSshHandle(obj)
            sshObj = obj.Ssh;
        end
        
        function [hasSudo, requiresPassword] = hasSudoAccess(obj)
        %hasSudoAccess Determines if a user has sudo access
        %   HASSUDO is TRUE if the user has sudo access and FALSE
        %   otherwise. If HASSUDO is TRUE, then REQUIRESPASSWORD
        %   indicates if sudo calls require a password or not. If
        %   REQUIRESPASSWORD is FALSE, it is password-less sudo.

        % Initialize default return values
            hasSudo = false;
            requiresPassword = true;

            % Try password-less sudo first (-n means non-interactive sudo)
            try
                obj.Ssh.execute('sudo -n true');
                hasSudo = true;
                requiresPassword = false;
                return;
            catch
                % Did not work
            end

            % Try sudo with password
            try
                obj.Ssh.execute(['echo ' obj.SudoPassword '| sudo -S true']);
                hasSudo = true;
                requiresPassword = true;
                return;
            catch
                % This did not work either. The user is probably not part
                % of the sudoer list.
            end
        end
        
        function sudoCmd = commandWithSudo(obj,command)
            %commandWithSudo Convert the given command to run with sudo if user has admin rights
            %   If the user has admin rights, prefix the sudo operation in
            %   front of the input command.
            %   If the user does not have admin rights, the command is
            %   returned verbatim.
            
            % By default, simple pass through the command
            sudoCmd = command;
            [hasSudo, requiresPw] = hasSudoAccess(obj);
            
            % By default, sudo -E does not preserve the shared library path
            % LD_LIBRARY_PATH
            sharedPath = 'LD_LIBRARY_PATH="$LD_LIBRARY_PATH"';
            
            hasSudo=false;
            
            if hasSudo
                % In both cases, make sure that the environment variables
                % of the parent session are preserved (-E option)
                if requiresPw
                    % Echo password to sudo invocation
                    sudoStr = ['echo ' obj.SudoPassword '| sudo ' sharedPath ' -E -S'];
                else
                    % Use non-interactive (-n) mode
                    sudoStr = ['sudo ' sharedPath ' -E -n'];
                end
                
                % Prefix the sudo directives to the command
                sudoCmd = [sudoStr ' ' command];
            end
        end
        

    end
    
    methods (Static)
        function p = fullfile(varargin)
            % Concatenate paths using Linux path separator
            p = varargin{1};
            for i = 2:nargin
                p = [p '/' varargin{i}]; %#ok<AGROW>
            end
        end
    end
end
