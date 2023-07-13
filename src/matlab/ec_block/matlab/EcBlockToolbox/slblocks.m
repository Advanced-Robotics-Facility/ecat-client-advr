function blkStruct = slblocks

blkStruct.OpenFcn = {'EcBlockToolbox'};
Browser(1).Library = 'EcBlockToolbox';
Browser(1).Name    = 'EcBlock Toolbox';

%%Browser.IsFlat  =  0;

blkStruct.Browser =  Browser;
