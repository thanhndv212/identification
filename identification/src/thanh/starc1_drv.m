function [] = starc1_drv(FILENAME, axe)

% Constantes des moteurs (couple = KT * courant)
KT=[1.03,1.03,0.57,0.24,0.24,0.24];

if nargin<1
    [FILENAME, PATHNAME] = uigetfile({'*.bin','Fichiers STARC1 (*.bin)'}, 'Sélection du fichier de données', '*.bin');
    if FILENAME==0
        error('Wrong file format')
    end
else
    if isempty(strfind(FILENAME, '/')) && ...
       isempty(strfind(FILENAME, '\\'))
        if isunix
            PATHNAME = [pwd, '/'];
        else
            PATHNAME = [pwd, '\\'];
        end
    else
        error('Cannot parse file name')
    end
end
if ~exist(FILENAME, 'file')
    error('File %s not found', FILENAME)
end
data = bt_readrecdata([PATHNAME,FILENAME]);

if nargin<2
    axe = choix_axe;
elseif ischar(axe)
    axe = str2num(axe);
end
if axe == 0
	pcmd_drv = []; for i = 1:6, pcmd_drv = [pcmd_drv bt_getrecdata(['pcmd' int2str(i)],1,data)]; end
	vcmd_drv = []; for i = 1:6, vcmd_drv = [vcmd_drv bt_getrecdata(['vcmd' int2str(i)],1,data)]; end
	acmd_drv = []; for i = 1:6, acmd_drv = [acmd_drv bt_getrecdata(['acmd' int2str(i)],1,data)]; end
	atc_drv  = []; for i = 1:6, atc_drv  = [atc_drv  KT(i)*bt_getrecdata(['atc' int2str(i)],1,data)]; end
	fcmd_drv = []; for i = 1:6, fcmd_drv = [fcmd_drv KT(i)*bt_getrecdata(['icmd' int2str(i)],1,data)]; end
	pfbk_drv = []; for i = 1:6, pfbk_drv = [pfbk_drv bt_getrecdata(['pfbk' int2str(i)],1,data)]; end
else
	pcmd_drv = bt_getrecdata(['pcmd' int2str(axe)],1,data);
	vcmd_drv = bt_getrecdata(['vcmd' int2str(axe)],1,data);
	acmd_drv = bt_getrecdata(['acmd' int2str(axe)],1,data);
	atc_drv  = bt_getrecdata(['atc' int2str(axe)],1,data);
	fcmd_drv = KT(axe)*bt_getrecdata(['icmd' int2str(axe)],1,data);
	pfbk_drv = bt_getrecdata(['pfbk' int2str(axe)],1,data);
end


figure(1)
plot(pcmd_drv)
if axe == 0,
	title('pcmd moteurs 1 à 6 en radians')
	legend('1','2','3','4','5','6',0)
else
	hold on
	plot(pfbk_drv,'r')
	hold off
	title(['pcmd et pfbk moteur ' int2str(axe) ' en radians'])
	legend('pcmd','pfbk',0)
end


figure(2)
plot(vcmd_drv)
if axe == 0,
	title('vcmd moteurs 1 à 6 en radians/s')
	legend('1','2','3','4','5','6',0)
else
	title(['vcmd moteur ' int2str(axe) ' en radians/s'])
end


figure(3)
plot(acmd_drv)
if axe == 0,
	title('acmd moteurs 1 à 6 en radians/s2')
	legend('1','2','3','4','5','6',0)
else
	title(['acmd moteur ' int2str(axe) ' en radians/s2'])
end


figure(4)
plot(atc_drv)
if axe == 0,
	title('atc moteurs 1 à 6 en N.m')
	legend('1','2','3','4','5','6',0)
else
	title(['atc moteur ' int2str(axe) ' en N.m'])
end


figure(5)
plot(fcmd_drv)
if axe == 0,
	title('fcmd moteurs 1 à 6 en N.m')
	legend('1','2','3','4','5','6',0)
else
	title(['fcmd moteur ' int2str(axe) ' en N.m'])
end


figure(6)
plot(pfbk_drv)
if axe == 0,
	title('pfbk moteurs 1 à 6 en radians')
	legend('1','2','3','4','5','6',0)
else
	title(['pfbk moteur ' int2str(axe) ' en radians'])
end
