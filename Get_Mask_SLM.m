function xyzp = Get_Mask_SLM(xyz,calibration)
% Get the mask from all points

% Data points
xy = xyz(:,1:2);
z = xyz(:,3);

% Calibration
scanZoom = calibration.ScanZoom;
imageSize = calibration.ImageSize;
scaleFactor = calibration.ScaleFactor;

% first part
FocusPlane = calibration.FocusPlane;
TransImage = calibration.TransImage;
% second part
% same FocusPlane
TransLaser = calibration.TransLaser;



% scale the xyImage coordinate based on the image pixel size (e.g. whether it is 512x512 or 256x256 image size, etc)
xy = xy*scaleFactor;

% Galvo center point to correct the target point on xy activation
% activation laser center point
activationCenter = imageSize/2;
% image laser center point, find the calibration file at 0 um plane
% [~, zPositionID]=min(abs(FocusPlane-0));
% imageCenter=tformfwd(TransImage{zPositionID}(end), activationCenter);

%imageCenter=[186 262];     % for 25X resonant scanner
imageCenter=[128 128];

% set(StatusWindow_text,'String',['Please place galvo at x=' num2str((imageCenter(1)*scaleFactor)) '; y=' num2str((imageCenter(2)*scaleFactor))]);
% uiwait(msgbox(['Please place galvo at x=' num2str((imageCenter(1)*scaleFactor)) '; y=' num2str((imageCenter(2)*scaleFactor)) '. When ready, press ENTER...']));
% correct for the scan zoom
xy(:,1)=(xy(:,1)-imageCenter(1)*scaleFactor)/scanZoom+imageCenter(1)*scaleFactor;
xy(:,2)=(xy(:,2)-imageCenter(2)*scaleFactor)/scanZoom+imageCenter(2)*scaleFactor;

% Construct 25 point zone
% 25 point zone
[zoneX,zoneY]=meshgrid(imageSize-imageSize/6:-imageSize/6:imageSize/6, ...
                       imageSize/6:imageSize/6:imageSize-imageSize/6);
zoneX=zoneX'; zoneX=zoneX(:);
zoneY=zoneY'; zoneY=zoneY(:);

% It is false for sure
stackTag = false; % JP
if stackTag      % activation stack, no need to do xyImage to xyActivation transfer
    xyActivation = xy;
else                % image stack, need to do a transform from xy image to xy activation    
% the following consider different focus for tImage2Activation, use 'linearinterp' for fitting
    xyActivation = zeros(size(xy));
    
    nPoints = size(xy,1);
    for i = 1:nPoints
        [~, zPositionID]=min(abs(FocusPlane-z(i)));
        [~, zoneID]=min((zoneX-xy(i,1)).^2+(zoneY-xy(i,2)).^2);
        
        if zPositionID==1
            tempzID = [1 2];
        else
            if zPositionID==length(FocusPlane)
                tempzID=[length(FocusPlane)-1 length(FocusPlane)];
            else
                tempzID=[zPositionID-1 zPositionID zPositionID+1];
            end
        end
        
        xyTemp = [];
        for j = 1:length(tempzID)
            xyTemp(j,:) = tforminv(TransImage{tempzID(j)}(zoneID),xy(i,:));
        end
        
        calFun1 = fit(FocusPlane(tempzID),xyTemp(:,1),'linearinterp' );
        calFun2 = fit(FocusPlane(tempzID),xyTemp(:,2),'linearinterp' );

        xyActivation(i,1)=calFun1(z(i));
        xyActivation(i,2)=calFun2(z(i));        
    end
end

% correct xy activation because we use the laser for activation instead of imaging
%xyActivationCorrected=xyActivation;
xyActivationCorrected(:,1)=imageSize+1-xyActivation(:,1);
xyActivationCorrected(:,2)=imageSize+1-xyActivation(:,2);

% transform from xy activation to xyp plane
xyzp=zeros(size(xyActivationCorrected));
                   
% the following consider different focus for tActivationLaserSLM, use 'linearinterp' for fitting 
for i=1:size(xyzp,1)
    [~, zPositionID]=min(abs(FocusPlane-z(i)));
    [~, zoneID]=min((zoneX-xyActivationCorrected(i,1)).^2+(zoneY-xyActivationCorrected(i,2)).^2);
    xyzp(i,:)=tforminv(TransLaser{zPositionID}(zoneID),xyActivationCorrected(i,:));
    
    if zPositionID==1
        tempzID=[1 2];
    else
        if zPositionID==length(FocusPlane)
            tempzID=[length(FocusPlane)-1 length(FocusPlane)];
        else
            tempzID=[zPositionID-1 zPositionID zPositionID+1];
        end
    end
            
    xyTemp = [];
    for j=1:length(tempzID)
        xyTemp(j,:)=tforminv(TransLaser{tempzID(j)}(zoneID),xyActivationCorrected(i,:));
    end
        
    calFun1 = fit( FocusPlane(tempzID), xyTemp(:,1), 'linearinterp' );
    calFun2 = fit( FocusPlane(tempzID), xyTemp(:,2), 'linearinterp' );

    xyzp(i,1)=calFun1(z(i));
    xyzp(i,2)=calFun2(z(i));        
end

xyzp(:,3) = z; 