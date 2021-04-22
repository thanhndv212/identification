% PARAMETRES - Déclaration des constantes constructeur
% pour les capteurs et
% des paramètres géométriques de la structure.

%     TX40 IRCCyN
%    Copyright (c) 2005 by IRCCyN, Maxime Gautier, PO Vandanjon
%		Création 10-06-2010
%    	Revision: 1.0  Date: 07-12-2012
%    	Revision: 1.0  Date: 03-06-2020 (erreur géométrique)

% accélération de la gravité,
% orientée positivement vers le centre de la terre
% projetée sur axe z0 vertical positif vers le ciel

% Ce script permet de calculer la matrice des fonctions de base : IDM
% Tau = IDM(q,qd,qdd)*X
%
% Le principe est de faire appel au script *_dim.m avec en entrée des
% positions, vitesses et accélérations aléatoire.
%
% on fait ensuite un balayage systématique de tous les paramètres possibles
% (14 par corps avec l'offset), on regarde ceux qui existent et cela, nous
% les mettons dans l'instruction

global g3
g3  = -9.8065;	%[m/s/s]

% nombre d'articulations
global narta ;
narta = 6 ;

% nombre de corps
global ncorps  ;
ncorps = 7 ;

%Paramètres géométriques de DH du TX40
global d3 rl1 rl3 rl4

% %parametres geometriques nominaux staubli
% d3=0.225;	%[m]
% rl1=0;	%[m], repère de base sur l'axe 2
% rl3=0.035;	%[m]
% rl4=0.225;	%[m]

% parametres geometriques modifiés pour étude sensibilité
% identification dynamique couple articulaire/erreur paramètres géométriques
% valeurs nominales
d3_nominal=0.225;     %[m]
rl1_nominal=0;        %[m], repère de base sur l'axe 2
rl3_nominal=0.035;	%[m]
rl4_nominal=0.225;	%[m]

% tau_erreur_geometrique=+400; %en pourcentage
tau_erreur_geometrique=0; %en pourcentage, valeurs nominales de référence
d3=d3_nominal*(1+tau_erreur_geometrique/100);	%[m]
rl1=rl1_nominal*(1+tau_erreur_geometrique/100);	%[m], repère de base sur l'axe 2
rl3=rl3_nominal*(1+tau_erreur_geometrique/100);	%[m]
rl4=rl4_nominal*(1+tau_erreur_geometrique/100);	%[m]

% d3=1;	%[m]
% rl1=0;	%[m], repère de base sur l'axe 2
% rl3=1;	%[m]
% rl4=1;	%[m]

% % paramètres de la charge nominale staubli
% % m7_pesee=1.70263; %(Kg) charge nominale staubli
% gt_ident=[27.7918 34.0115 29.2409 -11.52 18.48 7.68];
%(Kg) masses 2Kg+toolm, 2.21728+0.204
m7_pesee=2.4213;    %(Kg) masses 2Kg+porte outil
gt_ident=[30.2699 32.9278 26.6491 -11.52 18.48 7.68];

global  idmpar indice_parametre_inertiel

%rapports de reduction=vitesse rotor moteur/vitesse articulaire
% vitesses en colonne : qdmr=red*qd
% vitesses en ligne : qdmr'=qd'*red' et qd'=qdmr'*inv(red')

% calcul des variables articulaires :
% vitesses
% en colonne : qd_drv=red*qd_jnt, qd_jnt=inv(red)*qd_drv
% en ligne : qd_drv'=qd_jnt'*red', qd_jnt'=qd_drv'*inv(red')

% couple : conservation de la puissance, rendement réducteur=1
% en colonne
% c_jnt'*qd_jnt=c_drv'*qd_drv=c_drv'*red*qd_jnt
% c_jnt'=c_drv'*red, soit c_jnt=red'*c_drv
% en ligne : c_jnt'=c_drv'*red

%axe1
%red1=-32;  %doc staubli
red1=32;
%axe2
%red2=-32;  %doc staubli
red2=32;
%axe3
red3=45;
%axe4
%red4=48;  %doc staubli
red4=-48;
%axe5
red5=45;
%axe6
red6=32;

% réduction transmission, axes découplés
red_diag=diag([red1 red2 red3 red4 red5 red6]); %[rd_drv/rd_jnt]
% réduction transmission avec couplage axes 5 et 6
red=red_diag;
red(6,5)=red6;

invred=inv(red);

% données Staubli (tx40_irccyn_06_12_2006.xls)
% constantes de couple, données Staubli
kt=[1.03,1.03,0.57,0.24,0.24,0.24]; %Nm/A
kt_ident=gt_ident*invred;

% inertie rotor, côté moteur, données Staubli
ia_drv(1)=354.e-6;    %(kgm^2)
ia_drv(2)=354.e-6;    %(kgm^2)
ia_drv(3)=39.9e-6;   %(kgm^2)
ia_drv(4)=6.03e-6;   %(kgm^2)
ia_drv(5)=12.57e-6;  %(kgm^2)
ia_drv(6)=6.05e-6;   %(kgm^2)

% inertie rotor, côté articulaire
ia_jnt=ia_drv*red_diag*red_diag;

% cm_Nmjnt=icmd_A*gt
gt=kt*red;  %(Nm_jnt/A)

% gain d'actionnement identifiés
% gt=gt_ident;

% rendement dynamique du réducteur à vitesse et couple nominaux
% indépendant de la vitesse
% indépendant du couple
% couples en colonne :
% couple sortie réducteur=couple entrée*rend
% les mesures à un instant t sont rangées en ligne (couples)
% couple sortie réducteur'=couple entrée réducteur'*rend'
% essai 1 seule articulation libre, sans le couplage
% en colonne : c_drv_out=rend*c_drv
% prise en compte du couplage avec red
% en colonne : c_jnt=red'*c_drv_out=red'*rend*c_drv
% en ligne : c_jnt'=c_drv'*rend'*red

rend1=0.75;
rend2=0.75;
rend3=0.5;
rend4=0.5;
rend5=0.5;
rend6=0.63;
rend=diag([rend1 rend2 rend3 rend4 rend5 rend6]);

%butées articulaires

q1min_adm=-180*pi/180;		%[rd]
q1max_adm=+180*pi/180;		%[rd]

q2min_adm=-125*pi/180;	%[rd]
q2max_adm=+125*pi/180;	%[rd]

q3min_adm=-138*pi/180;	%[rd]
q3max_adm=+138*pi/180;	%[rd]

q4min_adm=-270*pi/180;		%[rd]
q4max_adm=+270*pi/180;		%[rd]

q5min_adm=-120*pi/180;          %[rd]
q5max_adm=+133.5*pi/180;    %[rd]

q6min_adm=-270*pi/180;          %[rd]
q6max_adm=+270*pi/180;         %[rd]

%vitesses articulaires admissibles (nominales)
qd1_adm=287*pi/180;                     %[rd_a/s]
qd2_adm=287*pi/180;                     %[rd_a/s]
qd3_adm=430*pi/180.;                    %[rd_a/s]
qd4_adm=410*pi/180;                     %[rd_a/s]
qd5_adm=320*pi/180;                     %[rd_a/s]
qd6_adm=700*pi/180;                     %[rd_a/s]
qdm6_adm=qd5_adm+qd6_adm;	%[rd_a/s]

%accélérations articulaires admissibles
qdd1_adm=1373*pi/180;                       %[rd_a/s/s]
qdd2_adm=1373*pi/180;                       %[rd_a/s/s]
qdd3_adm=3096*pi/180;                       %[rd_a/s/s]
qdd4_adm=2802*pi/180;                       %[rd_a/s/s]
qdd5_adm=1707*pi/180;                       %[rd_a/s/s]
qdd6_adm=8167*pi/180;                       %[rd_a/s/s]
qddm6_adm=qdd5_adm+qdd6_adm;	%[rd_a/s/s]

%Couples continus, rotor bloqué, Nm articulaire :
cm1_nom=abs(red(1,1))*1.85;	%[Nm_a]
cm2_nom=abs(red(2,2))*1.85;	%[Nm_a]
cm3_nom=abs(red(3,3))*0.68;	%[Nm_a]
cm4_nom=abs(red(4,4))*0.19;	%[Nm_a]
cm5_nom=abs(red(5,5))*0.19;	%[Nm_a]
cm6_nom=abs(red(6,6))*0.07;	%[Nm_a]

%Couples utile max sortie réducteur, Nm articulaire :
cm1_utile_adm=71.30;	%[Nm_a]
cm2_utile_adm=114.25;	%[Nm_a]
cm3_utile_adm=49.11;	%[Nm_a]
cm4_utile_adm=13.16;	%[Nm_a]
cm5_utile_adm=19.74;	%[Nm_a]
cm6_utile_adm=3.5;	%[Nm_a]
cm_utile_adm=...
    [cm1_utile_adm;cm2_utile_adm;cm3_utile_adm;...
    cm4_utile_adm;cm5_utile_adm;cm6_utile_adm];

%Couples crête max entrée réducteur, Nm articulaire :
cm1_in_red=abs(red(1,1))*4.12;	%[Nm_a]
cm2_in_red=abs(red(2,2))*4.12;	%[Nm_a]
cm3_in_red=abs(red(3,3))*2.30;	%[Nm_a]
cm4_in_red=abs(red(4,4))*0.96;	%[Nm_a]
cm5_in_red=abs(red(5,5))*0.96;	%[Nm_a]
cm6_in_red=abs(red(6,6))*0.96;	%[Nm_a]
cm_in_red=...
    [cm1_in_red;cm2_in_red;cm3_in_red;...
    cm4_in_red;cm5_in_red;cm6_in_red];

%couple de frottement au réducteur
cm_frot_red=abs(cm_in_red)-abs(cm_utile_adm);

% constantes pour la compensation de la gravité
% pas de compensation pour le TX
% r = 0.06 ; 		%mètre
% L = 0.390 ; 	%mètre
% AB = r+L ; 	%mètre
% Pc =1657.3 ;	% Newton
% kraid = 20995 ;% Newton/mètre
%
% alpha0 = 0 ; 	% base du robot sur le sol (robot non suspendu !!!)
% alpha0=180 degrés => robot suspendu

% Génération automatique du nom des paramètres standard
%à partir du modèle d'identification dynamique standard de SYMORO

% variables detectéé dans le modele dynamique
% dgnNomParm
% n 			: No du corps motorises
% NomPar 	: nom du parametres dynamiques : XX, MX, etc.
% m 			: No du corps considere :
% exemple 	: dg6fv6


nalea = 1 ;
% generation de variable aléatoire

for k=1:narta
    eval(['q' int2str(k) '= rand(nalea,1) ;']) ;
    eval(['qd' int2str(k) '= rand(nalea,1) ;']) ;
    eval(['qdd' int2str(k) '= rand(nalea,1) ;']) ;
end ;

qdm6= rand(nalea,1);
qddm6= rand(nalea,1);

% modele symoro du TX40
tx40s_charge_dim

% par =str2mat('xx ','xy ','xz ','yy ','yz ','zz ','mx ','my ','mz ','m  ','ia ', ...
    % 'fv ','fs ','fvm','fsm','off') ;
% parametres_inertiels =[ones(1,11), zeros(1,5)] ; 
%% vecteur qui distingue les parametres liés au frottement et ceux liés à l'inertie
% % c'est utile ensuite dans la fonction du modele direct ddm.m pour calculer
% % la matrice des masses

par =str2mat('xx ','xy ','xz ','yy ','yz ','zz ','mx ','my ','mz ','m  ','ia ', ...
    'fv ','fs ','off','iam','fvm','fsm') ;
parametres_inertiels =[ones(1,11), zeros(1,3),1,zeros(1,2)] ; 
% vecteur qui distingue les parametres liés au frottement et ceux liés à l'inertie
% c'est utile ensuite dans la fonction du modele direct ddm.m pour calculer
% la matrice des masses

% par = ['xx ';'xy ';'xz ';'yy ';'yz ';'zz ';'mx ';'my ';'mz ';'m  ';'ia '; ...
%     'fv ';'fs ';'off'] ;

% calcul de xpar
xpar = [] ;
indice_parametre_inertiel=[] ;
for l=1:ncorps
    for m=1:length(par)
        for k=1:narta
            variable_a_tester = ['dg' int2str(k), deblank(par(m,:)), int2str(l)] ;
            if exist(variable_a_tester, 'var')
                nom_du_parametre = [deblank(par(m,:)), int2str(l)] ;
                %if ~exist(nom_du_parametre,'var') verrue provenant du
                %passé
                if isempty(strmatch(nom_du_parametre,xpar))
                    % si le nom du paramètre n'est pas compris dans
                    % la liste des noms de paramètres, nous l'ajoutons
                    % sinon nous ne l'ajoutons pas
                    xpar = strvcat(xpar, nom_du_parametre) ;
                    if parametres_inertiels(m)
                        indice_parametre_inertiel = [indice_parametre_inertiel,size(xpar,1)] ;
                    end
                    %                     eval([nom_du_parametre, ' = 0 ;']) ;
                end
                %end;
            end ;
        end ;
    end ;
end;

% calcul de idmpar

npar = size(xpar,1) ; % nombre de ligne de xpar = nombre de parametre
idmpar =['idm = ['] ;
for m=1:narta
    for k=1:npar,
        variable_a_tester = ['dg' int2str(m), deblank(xpar(k,:))] ;
        if exist(variable_a_tester, 'var') % si la variable a été calculé par symoro
            idmpar =[idmpar, variable_a_tester,','] ;
        else
            idmpar =[idmpar, 'dgzero,'] ;
        end
        
    end % fin boucle sur les paramètres
    idmpar(end)= ';' ; % saut de ligne
end
idmpar(end)=']';
idmpar=[idmpar,';'];

%Paramètres de filtrage des signaux mesurés
%utilisés par scrip_filtre et filtre_donnee

% ordre du sous échantillonnage pour decimation(ndec)
% dans script_donnee\mesures_to_donnees
%ndec=20;   %pour starc1, 
            %sous échantillonnage de 5000Hz à 250Hz
ndec=1;     %pour cs8c, 250Hz
            %ou starc1, 5000Hz

%nombre de points (ordre) du filtrage médian
%pour élimination données aberrantes
% impair
nmedmf=3;
% nmedmf=1;   %pas de filtrage median            
            
% ordre du filtre de butterworth utilisé pour le filtrage
nbutter = 4 ;
%fréquence de coupure du Butterworth=0.8*fe/2/odmf
% odmf=2; %pour 250Hz  CS8C
% odmf=40; %pour 5000Hz starc1, coupure 50Hz
odmf=20; %pour 5000Hz starc1, coupure 100Hz
% odmf=4; %pour 5000Hz, coupure 500Hz
% odmf=6; %pour 5000Hz, coupure 333Hz

% odmf=1; %peu de filtrage
%odmf=ceil(odmf/ndec);

%parametres_filtrage=[nmedmf,nbutter,odmf, ndec];

% Paramètres pour QR itératif
% et filtrage parallèle
% utilisés dans scrip_obs

% Ordre du filtrage médian sur signe(qd) pour fs
% impair
nmed = 11;

% % avec signe(decimate(qd))
% %voir dans script_obs
% nmed=1; %pas de filtrage median

% Ordre du filtrage parallèle : fréq de coupure=0.8*fe/2/odmf
% ndreg=5;    %pour 250Hz
% ndreg=50;  %pour 250Hz et moyenne sur palier
ndreg=100;  %pour 5000Hz coupure 20Hz
% ndreg=4;  %pour 5000Hz, coupure 500Hz
% ndreg=6;  %pour 5000Hz, coupure 333Hz
% ndreg=1;  %sans filtrage
%ndreg = ceil(ndreg/ndec);

% Dimension des paquets pour le QR itératif
%dim_pack>ndreg*nombre de parametres standards
%pour avoir réduction d'un système surdéterminé
%dim_pack = 20000;
dim_pack = 1.e8;
if dim_pack<(ndreg*length(xpar))
    disp(['taille des paquets=',num2str(dim_pack),'insuffisante'])
    disp('modifier dim_pack dans parametres.m')
end


% %------------------- choix resolution+ seuillage vitesse------------

% %---------validation------------------
% resolution=0;	%validation
% %seuil vitesse=coef_seuil_vitesse*vit admissible
% % coef_seuil_vitesse=1.e-2;
% coef_seuil_vitesse=0.;
% %---------fin validation------------------

%---------resolution------------------
resolution=1;	%résolution
%seuil vitesse=coef_seuil_vitesse*vit admissible
coef_seuil_vitesse=1.e-2;   %pour essai mouvement 6 axes
% coef_seuil_vitesse=0.;    %pour essai axe par axe
%---------fin resolution------------------

%resolution = input('Résolution (1) ou validation (0): ');
% coef_seuil_vitesse=1.e-2;

%----------------fin choix resolution+ seuillage vitesse-------




%parametres_obs=[dim_pack,nmed,ndreg,coef_seuil_vitesse,resolution];

% Paramètres pour le calcul du modèle minimal
% utilisés par script_obs et modele_minimal

option_minimal=2; %valeurs en argument de modele_minimal

%ajuster le zero numérique pour le calcul du rang
tolerance_qr =2.e-2;

%ajuster le zero numérique pour le calcul des colonnes nulles
% et des seuils de 0 et 1 pour les coefficients des relations linéaires

% valeur minimale de la moyenne de la valeur absolue de chaque colonne de w
% pour élimination des paramètres sans effet sur le modèle
tolerance_zero = 1.e-3 ;

%parametres_modele_minimal=[option_minimal tolerance_qr tolerance_zero];

% seuil_essentiel sigma_e<seuil_essentiel*sigma_min
seuil_essentiel=1.03;
%seuil_essentiel=1.005;
%seuil_essentiel=1.000001;

%parametres essentiels
%max(ecart_type_relatif)<=ratio_essentiel*min(ecart_type_relatif)
% ratio_essentiel=20;
ratio_essentiel=30;
% ratio_essentiel=100000;   %pas de simplification, p de base

% sauvegarde des paramètres dans parametres_robot.mat
eval(['save parametres_robot narta ncorps',...
    ' d3_nominal rl1_nominal rl3_nominal rl4_nominal',...
    ' tau_erreur_geometrique d3 rl1 rl3 rl4',...
    ' red invred',...
    ' rend',...
    ' xpar idmpar g3 indice_parametre_inertiel',...
    ' q1min_adm q1max_adm qd1_adm qdd1_adm',...
    ' q2min_adm q2max_adm qd2_adm qdd2_adm',...
    ' q3min_adm q3max_adm qd3_adm qdd3_adm',...
    ' q4min_adm q4max_adm qd4_adm qdd4_adm',...
    ' q5min_adm q5max_adm qd5_adm qdd5_adm',...
    ' q6min_adm q6max_adm qd6_adm qdd6_adm',...
    '                    qdm6_adm qddm6_adm',...
    ' cm1_nom cm2_nom cm3_nom cm4_nom cm5_nom cm6_nom',...
    ' cm_utile_adm cm_in_red cm_frot_red',...
    ' nmedmf nbutter odmf ndec',...
    ' dim_pack nmed ndreg coef_seuil_vitesse resolution',...
    ' option_minimal tolerance_qr tolerance_zero',...
    ' seuil_essentiel ratio_essentiel']);

% PARAMETRES - Fin du programme