% (********************************************)
% (** symoro+ : symbolic modelling of robots **)
% (**========================================**)
% (**      irccyn-ecn - 1, rue de la noe     **)
% (**      b.p.92101                         **)
% (**      44321 nantes cedex 3, france      **)
% (**      www.irccyn.ec-nantes.fr           **)
% (********************************************)


%    name of file : m:\max_documents\dycaro_tx40_irccyn\symoro_tx40s\tx40s_charge.dim




%      geometric parameters   


% j        ant      mu       sigma    gamma    b        alpha    d        theta    r


% 1        0        1        0        0        0        0        0        q1       0

%                                                       -pi               -pi
% 2        1        1        0        0        0        ---      0        --- + q2 0
%                                                        2                 2
%                                                                         pi
% 3        2        1        0        0        0        0        d3       -- + q3  rl3
%                                                                         2
%                                                       pi
% 4        3        1        0        0        0        --       0        q4       rl4
%                                                       2
%                                                       -pi
% 5        4        1        0        0        0        ---      0        q5       0
%                                                        2
%                                                       pi
% 6        5        1        0        0        0        --       0        pi + q6  0
%                                                       2

% 7        6        0        2        0        0        0        0        0        0




%              inertial parameters

% j     xx    xy    xz    yy    yz    zz    mx    my    mz    m     ia

% 1     xx1   xy1   xz1   yy1   yz1   zz1   mx1   my1   mz1   m1    ia1

% 2     xx2   xy2   xz2   yy2   yz2   zz2   mx2   my2   mz2   m2    ia2

% 3     xx3   xy3   xz3   yy3   yz3   zz3   mx3   my3   mz3   m3    ia3

% 4     xx4   xy4   xz4   yy4   yz4   zz4   mx4   my4   mz4   m4    ia4

% 5     xx5   xy5   xz5   yy5   yz5   zz5   mx5   my5   mz5   m5    ia5

% 6     xx6   xy6   xz6   yy6   yz6   zz6   mx6   my6   mz6   m6    ia6

% 7     xx7   xy7   xz7   yy7   yz7   zz7   mx7   my7   mz7   m7    0



%  external forces,friction parameters, joint velocities and accelerations

% j      fx     fy     fz     cx     cy     cz     fs     fv     qp     qdp

% 1      0      0      0      0      0      0      fs1    fv1    qd1    qdd1

% 2      0      0      0      0      0      0      fs2    fv2    qd2    qdd2

% 3      0      0      0      0      0      0      fs3    fv3    qd3    qdd3

% 4      0      0      0      0      0      0      fs4    fv4    qd4    qdd4

% 5      0      0      0      0      0      0      fs5    fv5    qd5    qdd5

% 6      0      0      0      0      0      0      fs6    fv6    qd6    qdd6

% 7      0      0      0      0      0      0      0      0      0      0

% base velocity, base accelerations, and gravity

% j     w0    wp0   v0    vp0   g

% 1     0     0     0     0     0

% 2     0     0     0     0     0

% 3     0     0     0     0     g3

%    dynamic identification model  . 

% Passage de paramètres
global g3
global d3 rl3 rl4

% Début du modèle SYMORO

% equations:

% % declaration of the function
% function tx40s_charge_dim()
% 
% % declaration of global input variables
% global q2 q3 q4 q5 q6 qd1 qdd1 qd2 qdd2 g3
% global qd3 qdd3 d3 rl3 qd4 qdd4 rl4 qd5 qdd5 qd6
% global qdd6
% 
% % declaration of global output variables
% global dg1zz1 dg1ia1 dg1fv1 dg1fs1 dg2xx2 dg1xx2 dg2xy2 dg1xy2 dg2xz2 dg1xz2
% global dg2yy2 dg1yy2 dg2yz2 dg1yz2 dg2zz2 dg1zz2 dg2mx2 dg2my2 dg1mz2 dg2ia2
% global dg2fv2 dg2fs2 dg3xx3 dg2xx3 dg1xx3 dg3xy3 dg2xy3 dg1xy3 dg3xz3 dg2xz3
% global dg1xz3 dg3yy3 dg2yy3 dg1yy3 dg3yz3 dg2yz3 dg1yz3 dg3zz3 dg2zz3 dg1zz3
% global dg3mx3 dg2mx3 dg1mx3 dg3my3 dg2my3 dg1my3 dg2mz3 dg1mz3 dg2m3 dg1m3
% global dg3ia3 dg3fv3 dg3fs3 dg4xx4 dg3xx4 dg2xx4 dg1xx4 dg4xy4 dg3xy4 dg2xy4
% global dg1xy4 dg4xz4 dg3xz4 dg2xz4 dg1xz4 dg4yy4 dg3yy4 dg2yy4 dg1yy4 dg4yz4
% global dg3yz4 dg2yz4 dg1yz4 dg4zz4 dg3zz4 dg2zz4 dg1zz4 dg4mx4 dg3mx4 dg2mx4
% global dg1mx4 dg4my4 dg3my4 dg2my4 dg1my4 dg3mz4 dg2mz4 dg1mz4 dg3m4 dg2m4
% global dg1m4 dg4ia4 dg4fv4 dg4fs4 dg5xx5 dg4xx5 dg3xx5 dg2xx5 dg1xx5 dg5xy5
% global dg4xy5 dg3xy5 dg2xy5 dg1xy5 dg5xz5 dg4xz5 dg3xz5 dg2xz5 dg1xz5 dg5yy5
% global dg4yy5 dg3yy5 dg2yy5 dg1yy5 dg5yz5 dg4yz5 dg3yz5 dg2yz5 dg1yz5 dg5zz5
% global dg4zz5 dg3zz5 dg2zz5 dg1zz5 dg5mx5 dg4mx5 dg3mx5 dg2mx5 dg1mx5 dg5my5
% global dg4my5 dg3my5 dg2my5 dg1my5 dg4mz5 dg3mz5 dg2mz5 dg1mz5 dg3m5 dg2m5
% global dg1m5 dg5ia5 dg5fv5 dg5fs5 dg6xx6 dg5xx6 dg4xx6 dg3xx6 dg2xx6 dg1xx6
% global dg6xy6 dg5xy6 dg4xy6 dg3xy6 dg2xy6 dg1xy6 dg6xz6 dg5xz6 dg4xz6 dg3xz6
% global dg2xz6 dg1xz6 dg6yy6 dg5yy6 dg4yy6 dg3yy6 dg2yy6 dg1yy6 dg6yz6 dg5yz6
% global dg4yz6 dg3yz6 dg2yz6 dg1yz6 dg6zz6 dg5zz6 dg4zz6 dg3zz6 dg2zz6 dg1zz6
% global dg6mx6 dg5mx6 dg4mx6 dg3mx6 dg2mx6 dg1mx6 dg6my6 dg5my6 dg4my6 dg3my6
% global dg2my6 dg1my6 dg5mz6 dg4mz6 dg3mz6 dg2mz6 dg1mz6 dg3m6 dg2m6 dg1m6
% global dg6ia6 dg6fv6 dg6fs6 dg6xx7 dg5xx7 dg4xx7 dg3xx7 dg2xx7 dg1xx7 dg6xy7
% global dg5xy7 dg4xy7 dg3xy7 dg2xy7 dg1xy7 dg6xz7 dg5xz7 dg4xz7 dg3xz7 dg2xz7
% global dg1xz7 dg6yy7 dg5yy7 dg4yy7 dg3yy7 dg2yy7 dg1yy7 dg6yz7 dg5yz7 dg4yz7
% global dg3yz7 dg2yz7 dg1yz7 dg6zz7 dg5zz7 dg4zz7 dg3zz7 dg2zz7 dg1zz7 dg6mx7
% global dg5mx7 dg4mx7 dg3mx7 dg2mx7 dg1mx7 dg6my7 dg5my7 dg4my7 dg3my7 dg2my7
% global dg1my7 dg5mz7 dg4mz7 dg3mz7 dg2mz7 dg1mz7 dg3m7 dg2m7 dg1m7

% function description:

	s2=-cos(q2);
	c2=sin(q2);
	s3=cos(q3);
	c3=-sin(q3);
	s4=sin(q4);
	c4=cos(q4);
	s5=sin(q5);
	c5=cos(q5);
	s6=-sin(q6);
	c6=-cos(q6);
	wi12=-(qd1.*s2);
	wi22=-(c2.*qd1);
	wp12=-(qdd1.*s2) + qd2.*wi22;
	wp22=-(c2.*qdd1) - qd2.*wi12;
	dv112=-wi12.^2;
	dv222=-wi22.^2;
	dv332=-qd2.^2;
	dv122=wi12.*wi22;
	dv132=qd2.*wi12;
	dv232=qd2.*wi22;
	u112=dv222 + dv332;
	u122=dv122 - qdd2;
	u132=dv132 + wp22;
	u212=dv122 + qdd2;
	u232=dv232 - wp12;
	u312=dv132 - wp22;
	u322=dv232 + wp12;
	u332=dv112 + dv222;
	vp12=g3.*s2;
	vp22=c2.*g3;
	wi13=c3.*wi12 + s3.*wi22;
	wi23=-(s3.*wi12) + c3.*wi22;
	w33=qd2 + qd3;
	wp13=qd3.*wi23 + c3.*wp12 + s3.*wp22;
	wp23=-(qd3.*wi13) - s3.*wp12 + c3.*wp22;
	wp33=qdd2 + qdd3;
	dv113=-wi13.^2;
	dv223=-wi23.^2;
	dv333=-w33.^2;
	dv123=wi13.*wi23;
	dv133=w33.*wi13;
	dv233=w33.*wi23;
	u113=dv223 + dv333;
	u123=dv123 - wp33;
	u133=dv133 + wp23;
	u213=dv123 + wp33;
	u223=dv113 + dv333;
	u233=dv233 - wp13;
	u313=dv133 - wp23;
	u323=dv233 + wp13;
	u333=dv113 + dv223;
	vsp13=d3.*u112 + rl3.*u132 + vp12;
	vsp23=d3.*u212 + rl3.*u232 + vp22;
	vsp33=d3.*u312 + rl3.*u332;
	vp13=c3.*vsp13 + s3.*vsp23;
	vp23=-(s3.*vsp13) + c3.*vsp23;
	wi14=s4.*w33 + c4.*wi13;
	wi24=c4.*w33 - s4.*wi13;
	w34=qd4 - wi23;
	wp14=qd4.*wi24 + c4.*wp13 + s4.*wp33;
	wp24=-(qd4.*wi14) - s4.*wp13 + c4.*wp33;
	wp34=qdd4 - wp23;
	dv114=-wi14.^2;
	dv224=-wi24.^2;
	dv334=-w34.^2;
	dv124=wi14.*wi24;
	dv134=w34.*wi14;
	dv234=w34.*wi24;
	u114=dv224 + dv334;
	u124=dv124 - wp34;
	u134=dv134 + wp24;
	u214=dv124 + wp34;
	u224=dv114 + dv334;
	u234=dv234 - wp14;
	u314=dv134 - wp24;
	u324=dv234 + wp14;
	u334=dv114 + dv224;
	vsp14=-(rl4.*u123) + vp13;
	vsp24=-(rl4.*u223) + vp23;
	vsp34=-(rl4.*u323) + vsp33;
	vp14=c4.*vsp14 + s4.*vsp34;
	vp24=-(s4.*vsp14) + c4.*vsp34;
	wi15=-(s5.*w34) + c5.*wi14;
	wi25=-(c5.*w34) - s5.*wi14;
	w35=qd5 + wi24;
	wp15=qd5.*wi25 + c5.*wp14 - s5.*wp34;
	wp25=-(qd5.*wi15) - s5.*wp14 - c5.*wp34;
	wp35=qdd5 + wp24;
	dv115=-wi15.^2;
	dv225=-wi25.^2;
	dv335=-w35.^2;
	dv125=wi15.*wi25;
	dv135=w35.*wi15;
	dv235=w35.*wi25;
	u115=dv225 + dv335;
	u125=dv125 - wp35;
	u135=dv135 + wp25;
	u215=dv125 + wp35;
	u225=dv115 + dv335;
	u235=dv235 - wp15;
	u315=dv135 - wp25;
	u325=dv235 + wp15;
	u335=dv115 + dv225;
	vp15=c5.*vp14 + s5.*vsp24;
	vp25=-(s5.*vp14) + c5.*vsp24;
	wi16=s6.*w35 + c6.*wi15;
	wi26=c6.*w35 - s6.*wi15;
	w36=qd6 - wi25;
	wp16=qd6.*wi26 + c6.*wp15 + s6.*wp35;
	wp26=-(qd6.*wi16) - s6.*wp15 + c6.*wp35;
	wp36=qdd6 - wp25;
	dv116=-wi16.^2;
	dv226=-wi26.^2;
	dv336=-w36.^2;
	dv126=wi16.*wi26;
	dv136=w36.*wi16;
	dv236=w36.*wi26;
	u116=dv226 + dv336;
	u126=dv126 - wp36;
	u136=dv136 + wp26;
	u216=dv126 + wp36;
	u226=dv116 + dv336;
	u236=dv236 - wp16;
	u316=dv136 - wp26;
	u326=dv236 + wp16;
	u336=dv116 + dv226;
	vp16=c6.*vp15 + s6.*vp24;
	vp26=-(s6.*vp15) + c6.*vp24;
	dv117=-wi16.^2;
	dv227=-wi26.^2;
	dv337=-w36.^2;
	dv127=wi16.*wi26;
	dv137=w36.*wi16;
	dv237=w36.*wi26;
	u117=dv227 + dv337;
	u127=dv127 - wp36;
	u137=dv137 + wp26;
	u217=dv127 + wp36;
	u227=dv117 + dv337;
	u237=dv237 - wp16;
	u317=dv137 - wp26;
	u327=dv237 + wp16;
	u337=dv117 + dv227;
	dg1zz1=qdd1;
	dg1ia1=qdd1;
	dg1fv1=qd1;
	dg1fs1=sign(qd1);
	dg2xx2=-dv122;
	n1xx23=-(c2.*dv132) - s2.*wp12;
	dg1xx2=n1xx23;
	no2xy23=-dv112 + dv222;
	dg2xy2=no2xy23;
	n1xy23=s2.*u312 - c2.*u322;
	dg1xy2=n1xy23;
	no2xz22=dv112 - dv332;
	dg2xz2=-u232;
	n1xz23=-(c2.*no2xz22) - s2.*u212;
	dg1xz2=n1xz23;
	dg2yy2=dv122;
	n1yy23=dv232.*s2 - c2.*wp22;
	dg1yy2=n1yy23;
	no2yz21=-dv222 + dv332;
	dg2yz2=u132;
	n1yz23=-(no2yz21.*s2) + c2.*u122;
	dg1yz2=n1yz23;
	dg2zz2=qdd2;
	n1zz23=c2.*dv132 - dv232.*s2;
	dg1zz2=n1zz23;
	dg2mx2=vp22;
	dg2my2=-vp12;
	n1mz23=-(c2.*vp12) + s2.*vp22;
	dg1mz2=n1mz23;
	dg2ia2=qdd2;
	dg2fv2=qd2;
	dg2fs2=sign(qd2);
	dg3xx3=-dv123;
	n2xx31=-(dv133.*s3) + c3.*wp13;
	n2xx32=c3.*dv133 + s3.*wp13;
	dg2xx3=-dv123;
	n1xx33=-(c2.*n2xx32) - n2xx31.*s2;
	dg1xx3=n1xx33;
	no3xy33=-dv113 + dv223;
	dg3xy3=no3xy33;
	n2xy31=-(c3.*u313) - s3.*u323;
	n2xy32=-(s3.*u313) + c3.*u323;
	dg2xy3=no3xy33;
	n1xy33=-(c2.*n2xy32) - n2xy31.*s2;
	dg1xy3=n1xy33;
	no3xz32=dv113 - dv333;
	dg3xz3=-u233;
	n2xz31=-(no3xz32.*s3) + c3.*u213;
	n2xz32=c3.*no3xz32 + s3.*u213;
	dg2xz3=-u233;
	n1xz33=-(c2.*n2xz32) - n2xz31.*s2;
	dg1xz3=n1xz33;
	dg3yy3=dv123;
	n2yy31=-(c3.*dv233) - s3.*wp23;
	n2yy32=-(dv233.*s3) + c3.*wp23;
	dg2yy3=dv123;
	n1yy33=-(c2.*n2yy32) - n2yy31.*s2;
	dg1yy3=n1yy33;
	no3yz31=-dv223 + dv333;
	dg3yz3=u133;
	n2yz31=c3.*no3yz31 + s3.*u123;
	n2yz32=no3yz31.*s3 - c3.*u123;
	dg2yz3=u133;
	n1yz33=-(c2.*n2yz32) - n2yz31.*s2;
	dg1yz3=n1yz33;
	dg3zz3=wp33;
	n2zz31=c3.*dv233 + dv133.*s3;
	n2zz32=-(c3.*dv133) + dv233.*s3;
	dg2zz3=wp33;
	n1zz33=-(c2.*n2zz32) - n2zz31.*s2;
	dg1zz3=n1zz33;
	dg3mx3=vp23;
	n2mx31=-(rl3.*(s3.*u113 + c3.*u213)) + s3.*vsp33;
	n2mx32=rl3.*(c3.*u113 - s3.*u213) - d3.*u313 - c3.*vsp33;
	n2mx33=d3.*(s3.*u113 + c3.*u213) + vp23;
	dg2mx3=n2mx33;
	n1mx33=-(c2.*n2mx32) - n2mx31.*s2;
	dg1mx3=n1mx33;
	dg3my3=-vp13;
	n2my31=-(rl3.*(s3.*u123 + c3.*u223)) + c3.*vsp33;
	n2my32=rl3.*(c3.*u123 - s3.*u223) - d3.*u323 + s3.*vsp33;
	n2my33=d3.*(s3.*u123 + c3.*u223) - vp13;
	dg2my3=n2my33;
	n1my33=-(c2.*n2my32) - n2my31.*s2;
	dg1my3=n1my33;
	n2mz31=-(rl3.*(s3.*u133 + c3.*u233)) - s3.*vp13 - c3.*vp23;
	n2mz32=rl3.*(c3.*u133 - s3.*u233) - d3.*u333 + c3.*vp13 - s3.*vp23;
	n2mz33=d3.*(s3.*u133 + c3.*u233);
	dg2mz3=n2mz33;
	n1mz33=-(c2.*n2mz32) - n2mz31.*s2;
	dg1mz3=n1mz33;
	n2m31=-(rl3.*(s3.*vp13 + c3.*vp23));
	n2m32=rl3.*(c3.*vp13 - s3.*vp23) - d3.*vsp33;
	n2m33=d3.*(s3.*vp13 + c3.*vp23);
	dg2m3=n2m33;
	n1m33=-(c2.*n2m32) - n2m31.*s2;
	dg1m3=n1m33;
	dg3ia3=qdd3;
	dg3fv3=qd3;
	dg3fs3=sign(qd3);
	dg4xx4=-dv124;
	n3xx41=-(dv134.*s4) + c4.*wp14;
	n3xx43=c4.*dv134 + s4.*wp14;
	dg3xx4=n3xx43;
	n2xx41=c3.*n3xx41 - dv124.*s3;
	n2xx42=c3.*dv124 + n3xx41.*s3;
	dg2xx4=n3xx43;
	n1xx43=-(c2.*n2xx42) - n2xx41.*s2;
	dg1xx4=n1xx43;
	no4xy43=-dv114 + dv224;
	dg4xy4=no4xy43;
	n3xy41=-(c4.*u314) - s4.*u324;
	n3xy43=-(s4.*u314) + c4.*u324;
	dg3xy4=n3xy43;
	n2xy41=c3.*n3xy41 + no4xy43.*s3;
	n2xy42=-(c3.*no4xy43) + n3xy41.*s3;
	dg2xy4=n3xy43;
	n1xy43=-(c2.*n2xy42) - n2xy41.*s2;
	dg1xy4=n1xy43;
	no4xz42=dv114 - dv334;
	dg4xz4=-u234;
	n3xz41=-(no4xz42.*s4) + c4.*u214;
	n3xz43=c4.*no4xz42 + s4.*u214;
	dg3xz4=n3xz43;
	n2xz41=c3.*n3xz41 - s3.*u234;
	n2xz42=n3xz41.*s3 + c3.*u234;
	dg2xz4=n3xz43;
	n1xz43=-(c2.*n2xz42) - n2xz41.*s2;
	dg1xz4=n1xz43;
	dg4yy4=dv124;
	n3yy41=-(c4.*dv234) - s4.*wp24;
	n3yy43=-(dv234.*s4) + c4.*wp24;
	dg3yy4=n3yy43;
	n2yy41=c3.*n3yy41 + dv124.*s3;
	n2yy42=-(c3.*dv124) + n3yy41.*s3;
	dg2yy4=n3yy43;
	n1yy43=-(c2.*n2yy42) - n2yy41.*s2;
	dg1yy4=n1yy43;
	no4yz41=-dv224 + dv334;
	dg4yz4=u134;
	n3yz41=c4.*no4yz41 + s4.*u124;
	n3yz43=no4yz41.*s4 - c4.*u124;
	dg3yz4=n3yz43;
	n2yz41=c3.*n3yz41 + s3.*u134;
	n2yz42=n3yz41.*s3 - c3.*u134;
	dg2yz4=n3yz43;
	n1yz43=-(c2.*n2yz42) - n2yz41.*s2;
	dg1yz4=n1yz43;
	dg4zz4=wp34;
	n3zz41=c4.*dv234 + dv134.*s4;
	n3zz43=-(c4.*dv134) + dv234.*s4;
	dg3zz4=n3zz43;
	n2zz41=c3.*n3zz41 + s3.*wp34;
	n2zz42=n3zz41.*s3 - c3.*wp34;
	dg2zz4=n3zz43;
	n1zz43=-(c2.*n2zz42) - n2zz41.*s2;
	dg1zz4=n1zz43;
	dg4mx4=vp24;
	e3mx41=c4.*u114 - s4.*u214;
	e3mx43=s4.*u114 + c4.*u214;
	n3mx41=-(rl4.*(s4.*u114 + c4.*u214)) - s4.*vsp24;
	n3mx43=rl4.*(c4.*u114 - s4.*u214) + c4.*vsp24;
	dg3mx4=n3mx43;
	n2mx41=c3.*n3mx41 - rl3.*(e3mx41.*s3 - c3.*u314) + s3.*vp24;
	n2mx42=-(d3.*e3mx43) + n3mx41.*s3 + rl3.*(c3.*e3mx41 + s3.*u314) - c3.*vp24;
	n2mx43=n3mx43 + d3.*(e3mx41.*s3 - c3.*u314);
	dg2mx4=n2mx43;
	n1mx43=-(c2.*n2mx42) - n2mx41.*s2;
	dg1mx4=n1mx43;
	dg4my4=-vp14;
	e3my41=c4.*u124 - s4.*u224;
	e3my43=s4.*u124 + c4.*u224;
	n3my41=-(rl4.*(s4.*u124 + c4.*u224)) - c4.*vsp24;
	n3my43=rl4.*(c4.*u124 - s4.*u224) - s4.*vsp24;
	dg3my4=n3my43;
	n2my41=c3.*n3my41 - rl3.*(e3my41.*s3 - c3.*u324) - s3.*vp14;
	n2my42=-(d3.*e3my43) + n3my41.*s3 + rl3.*(c3.*e3my41 + s3.*u324) + c3.*vp14;
	n2my43=n3my43 + d3.*(e3my41.*s3 - c3.*u324);
	dg2my4=n2my43;
	n1my43=-(c2.*n2my42) - n2my41.*s2;
	dg1my4=n1my43;
	e3mz41=c4.*u134 - s4.*u234;
	e3mz43=s4.*u134 + c4.*u234;
	n3mz41=-(rl4.*(s4.*u134 + c4.*u234)) - s4.*vp14 - c4.*vp24;
	n3mz43=rl4.*(c4.*u134 - s4.*u234) + c4.*vp14 - s4.*vp24;
	dg3mz4=n3mz43;
	n2mz41=c3.*n3mz41 - rl3.*(e3mz41.*s3 - c3.*u334);
	n2mz42=-(d3.*e3mz43) + n3mz41.*s3 + rl3.*(c3.*e3mz41 + s3.*u334);
	n2mz43=n3mz43 + d3.*(e3mz41.*s3 - c3.*u334);
	dg2mz4=n2mz43;
	n1mz43=-(c2.*n2mz42) - n2mz41.*s2;
	dg1mz4=n1mz43;
	e3m41=c4.*vp14 - s4.*vp24;
	e3m43=s4.*vp14 + c4.*vp24;
	n3m41=-(rl4.*(s4.*vp14 + c4.*vp24));
	n3m43=rl4.*(c4.*vp14 - s4.*vp24);
	dg3m4=n3m43;
	n2m41=c3.*n3m41 - rl3.*(e3m41.*s3 + c3.*vsp24);
	n2m42=-(d3.*e3m43) + n3m41.*s3 + rl3.*(c3.*e3m41 - s3.*vsp24);
	n2m43=n3m43 + d3.*(e3m41.*s3 + c3.*vsp24);
	dg2m4=n2m43;
	n1m43=-(c2.*n2m42) - n2m41.*s2;
	dg1m4=n1m43;
	dg4ia4=qdd4;
	dg4fv4=qd4;
	dg4fs4=sign(qd4);
	dg5xx5=-dv125;
	n4xx51=-(dv135.*s5) + c5.*wp15;
	n4xx53=-(c5.*dv135) - s5.*wp15;
	dg4xx5=n4xx53;
	n3xx51=c4.*n4xx51 + dv125.*s4;
	n3xx53=-(c4.*dv125) + n4xx51.*s4;
	dg3xx5=n3xx53;
	n2xx51=c3.*n3xx51 + n4xx53.*s3;
	n2xx52=-(c3.*n4xx53) + n3xx51.*s3;
	dg2xx5=n3xx53;
	n1xx53=-(c2.*n2xx52) - n2xx51.*s2;
	dg1xx5=n1xx53;
	no5xy53=-dv115 + dv225;
	dg5xy5=no5xy53;
	n4xy51=-(c5.*u315) - s5.*u325;
	n4xy53=s5.*u315 - c5.*u325;
	dg4xy5=n4xy53;
	n3xy51=c4.*n4xy51 - no5xy53.*s4;
	n3xy53=c4.*no5xy53 + n4xy51.*s4;
	dg3xy5=n3xy53;
	n2xy51=c3.*n3xy51 + n4xy53.*s3;
	n2xy52=-(c3.*n4xy53) + n3xy51.*s3;
	dg2xy5=n3xy53;
	n1xy53=-(c2.*n2xy52) - n2xy51.*s2;
	dg1xy5=n1xy53;
	no5xz52=dv115 - dv335;
	dg5xz5=-u235;
	n4xz51=-(no5xz52.*s5) + c5.*u215;
	n4xz53=-(c5.*no5xz52) - s5.*u215;
	dg4xz5=n4xz53;
	n3xz51=c4.*n4xz51 + s4.*u235;
	n3xz53=n4xz51.*s4 - c4.*u235;
	dg3xz5=n3xz53;
	n2xz51=c3.*n3xz51 + n4xz53.*s3;
	n2xz52=-(c3.*n4xz53) + n3xz51.*s3;
	dg2xz5=n3xz53;
	n1xz53=-(c2.*n2xz52) - n2xz51.*s2;
	dg1xz5=n1xz53;
	dg5yy5=dv125;
	n4yy51=-(c5.*dv235) - s5.*wp25;
	n4yy53=dv235.*s5 - c5.*wp25;
	dg4yy5=n4yy53;
	n3yy51=c4.*n4yy51 - dv125.*s4;
	n3yy53=c4.*dv125 + n4yy51.*s4;
	dg3yy5=n3yy53;
	n2yy51=c3.*n3yy51 + n4yy53.*s3;
	n2yy52=-(c3.*n4yy53) + n3yy51.*s3;
	dg2yy5=n3yy53;
	n1yy53=-(c2.*n2yy52) - n2yy51.*s2;
	dg1yy5=n1yy53;
	no5yz51=-dv225 + dv335;
	dg5yz5=u135;
	n4yz51=c5.*no5yz51 + s5.*u125;
	n4yz53=-(no5yz51.*s5) + c5.*u125;
	dg4yz5=n4yz53;
	n3yz51=c4.*n4yz51 - s4.*u135;
	n3yz53=n4yz51.*s4 + c4.*u135;
	dg3yz5=n3yz53;
	n2yz51=c3.*n3yz51 + n4yz53.*s3;
	n2yz52=-(c3.*n4yz53) + n3yz51.*s3;
	dg2yz5=n3yz53;
	n1yz53=-(c2.*n2yz52) - n2yz51.*s2;
	dg1yz5=n1yz53;
	dg5zz5=wp35;
	n4zz51=c5.*dv235 + dv135.*s5;
	n4zz53=c5.*dv135 - dv235.*s5;
	dg4zz5=n4zz53;
	n3zz51=c4.*n4zz51 - s4.*wp35;
	n3zz53=n4zz51.*s4 + c4.*wp35;
	dg3zz5=n3zz53;
	n2zz51=c3.*n3zz51 + n4zz53.*s3;
	n2zz52=-(c3.*n4zz53) + n3zz51.*s3;
	dg2zz5=n3zz53;
	n1zz53=-(c2.*n2zz52) - n2zz51.*s2;
	dg1zz5=n1zz53;
	dg5mx5=vp25;
	e4mx51=c5.*u115 - s5.*u215;
	e4mx53=-(s5.*u115) - c5.*u215;
	n4mx51=s5.*vp24;
	n4mx53=c5.*vp24;
	dg4mx5=n4mx53;
	e3mx51=c4.*e4mx51 - s4.*u315;
	e3mx53=e4mx51.*s4 + c4.*u315;
	n3mx51=c4.*n4mx51 - rl4.*(e4mx51.*s4 + c4.*u315) - s4.*vp25;
	n3mx53=n4mx51.*s4 + rl4.*(c4.*e4mx51 - s4.*u315) + c4.*vp25;
	dg3mx5=n3mx53;
	n2mx51=c3.*n3mx51 + n4mx53.*s3 - rl3.*(-(c3.*e4mx53) + e3mx51.*s3);
	n2mx52=-(d3.*e3mx53) - c3.*n4mx53 + n3mx51.*s3 + rl3.*(c3.*e3mx51 + e4mx53.*s3);
	n2mx53=n3mx53 + d3.*(-(c3.*e4mx53) + e3mx51.*s3);
	dg2mx5=n2mx53;
	n1mx53=-(c2.*n2mx52) - n2mx51.*s2;
	dg1mx5=n1mx53;
	dg5my5=-vp15;
	e4my51=c5.*u125 - s5.*u225;
	e4my53=-(s5.*u125) - c5.*u225;
	n4my51=c5.*vp24;
	n4my53=-(s5.*vp24);
	dg4my5=n4my53;
	e3my51=c4.*e4my51 - s4.*u325;
	e3my53=e4my51.*s4 + c4.*u325;
	n3my51=c4.*n4my51 - rl4.*(e4my51.*s4 + c4.*u325) + s4.*vp15;
	n3my53=n4my51.*s4 + rl4.*(c4.*e4my51 - s4.*u325) - c4.*vp15;
	dg3my5=n3my53;
	n2my51=c3.*n3my51 + n4my53.*s3 - rl3.*(-(c3.*e4my53) + e3my51.*s3);
	n2my52=-(d3.*e3my53) - c3.*n4my53 + n3my51.*s3 + rl3.*(c3.*e3my51 + e4my53.*s3);
	n2my53=n3my53 + d3.*(-(c3.*e4my53) + e3my51.*s3);
	dg2my5=n2my53;
	n1my53=-(c2.*n2my52) - n2my51.*s2;
	dg1my5=n1my53;
	e4mz51=c5.*u135 - s5.*u235;
	e4mz53=-(s5.*u135) - c5.*u235;
	n4mz51=-(s5.*vp15) - c5.*vp25;
	n4mz53=-(c5.*vp15) + s5.*vp25;
	dg4mz5=n4mz53;
	e3mz51=c4.*e4mz51 - s4.*u335;
	e3mz53=e4mz51.*s4 + c4.*u335;
	n3mz51=c4.*n4mz51 - rl4.*(e4mz51.*s4 + c4.*u335);
	n3mz53=n4mz51.*s4 + rl4.*(c4.*e4mz51 - s4.*u335);
	dg3mz5=n3mz53;
	n2mz51=c3.*n3mz51 + n4mz53.*s3 - rl3.*(-(c3.*e4mz53) + e3mz51.*s3);
	n2mz52=-(d3.*e3mz53) - c3.*n4mz53 + n3mz51.*s3 + rl3.*(c3.*e3mz51 + e4mz53.*s3);
	n2mz53=n3mz53 + d3.*(-(c3.*e4mz53) + e3mz51.*s3);
	dg2mz5=n2mz53;
	n1mz53=-(c2.*n2mz52) - n2mz51.*s2;
	dg1mz5=n1mz53;
	e4m51=c5.*vp15 - s5.*vp25;
	e4m53=-(s5.*vp15) - c5.*vp25;
	e3m51=c4.*e4m51 - s4.*vp24;
	e3m53=e4m51.*s4 + c4.*vp24;
	n3m51=-(rl4.*(e4m51.*s4 + c4.*vp24));
	n3m53=rl4.*(c4.*e4m51 - s4.*vp24);
	dg3m5=n3m53;
	n2m51=c3.*n3m51 - rl3.*(-(c3.*e4m53) + e3m51.*s3);
	n2m52=-(d3.*e3m53) + n3m51.*s3 + rl3.*(c3.*e3m51 + e4m53.*s3);
	n2m53=n3m53 + d3.*(-(c3.*e4m53) + e3m51.*s3);
	dg2m5=n2m53;
	n1m53=-(c2.*n2m52) - n2m51.*s2;
	dg1m5=n1m53;
	dg5ia5=qdd5;
	dg5fv5=qd5;
	dg5fs5=sign(qd5);
	dg6xx6=-dv126;
	n5xx61=-(dv136.*s6) + c6.*wp16;
	n5xx63=c6.*dv136 + s6.*wp16;
	dg5xx6=n5xx63;
	n4xx61=c5.*n5xx61 - dv126.*s5;
	n4xx63=-(c5.*dv126) - n5xx61.*s5;
	dg4xx6=n4xx63;
	n3xx61=c4.*n4xx61 - n5xx63.*s4;
	n3xx63=c4.*n5xx63 + n4xx61.*s4;
	dg3xx6=n3xx63;
	n2xx61=c3.*n3xx61 + n4xx63.*s3;
	n2xx62=-(c3.*n4xx63) + n3xx61.*s3;
	dg2xx6=n3xx63;
	n1xx63=-(c2.*n2xx62) - n2xx61.*s2;
	dg1xx6=n1xx63;
	no6xy63=-dv116 + dv226;
	dg6xy6=no6xy63;
	n5xy61=-(c6.*u316) - s6.*u326;
	n5xy63=-(s6.*u316) + c6.*u326;
	dg5xy6=n5xy63;
	n4xy61=c5.*n5xy61 + no6xy63.*s5;
	n4xy63=c5.*no6xy63 - n5xy61.*s5;
	dg4xy6=n4xy63;
	n3xy61=c4.*n4xy61 - n5xy63.*s4;
	n3xy63=c4.*n5xy63 + n4xy61.*s4;
	dg3xy6=n3xy63;
	n2xy61=c3.*n3xy61 + n4xy63.*s3;
	n2xy62=-(c3.*n4xy63) + n3xy61.*s3;
	dg2xy6=n3xy63;
	n1xy63=-(c2.*n2xy62) - n2xy61.*s2;
	dg1xy6=n1xy63;
	no6xz62=dv116 - dv336;
	dg6xz6=-u236;
	n5xz61=-(no6xz62.*s6) + c6.*u216;
	n5xz63=c6.*no6xz62 + s6.*u216;
	dg5xz6=n5xz63;
	n4xz61=c5.*n5xz61 - s5.*u236;
	n4xz63=-(n5xz61.*s5) - c5.*u236;
	dg4xz6=n4xz63;
	n3xz61=c4.*n4xz61 - n5xz63.*s4;
	n3xz63=c4.*n5xz63 + n4xz61.*s4;
	dg3xz6=n3xz63;
	n2xz61=c3.*n3xz61 + n4xz63.*s3;
	n2xz62=-(c3.*n4xz63) + n3xz61.*s3;
	dg2xz6=n3xz63;
	n1xz63=-(c2.*n2xz62) - n2xz61.*s2;
	dg1xz6=n1xz63;
	dg6yy6=dv126;
	n5yy61=-(c6.*dv236) - s6.*wp26;
	n5yy63=-(dv236.*s6) + c6.*wp26;
	dg5yy6=n5yy63;
	n4yy61=c5.*n5yy61 + dv126.*s5;
	n4yy63=c5.*dv126 - n5yy61.*s5;
	dg4yy6=n4yy63;
	n3yy61=c4.*n4yy61 - n5yy63.*s4;
	n3yy63=c4.*n5yy63 + n4yy61.*s4;
	dg3yy6=n3yy63;
	n2yy61=c3.*n3yy61 + n4yy63.*s3;
	n2yy62=-(c3.*n4yy63) + n3yy61.*s3;
	dg2yy6=n3yy63;
	n1yy63=-(c2.*n2yy62) - n2yy61.*s2;
	dg1yy6=n1yy63;
	no6yz61=-dv226 + dv336;
	dg6yz6=u136;
	n5yz61=c6.*no6yz61 + s6.*u126;
	n5yz63=no6yz61.*s6 - c6.*u126;
	dg5yz6=n5yz63;
	n4yz61=c5.*n5yz61 + s5.*u136;
	n4yz63=-(n5yz61.*s5) + c5.*u136;
	dg4yz6=n4yz63;
	n3yz61=c4.*n4yz61 - n5yz63.*s4;
	n3yz63=c4.*n5yz63 + n4yz61.*s4;
	dg3yz6=n3yz63;
	n2yz61=c3.*n3yz61 + n4yz63.*s3;
	n2yz62=-(c3.*n4yz63) + n3yz61.*s3;
	dg2yz6=n3yz63;
	n1yz63=-(c2.*n2yz62) - n2yz61.*s2;
	dg1yz6=n1yz63;
	dg6zz6=wp36;
	n5zz61=c6.*dv236 + dv136.*s6;
	n5zz63=-(c6.*dv136) + dv236.*s6;
	dg5zz6=n5zz63;
	n4zz61=c5.*n5zz61 + s5.*wp36;
	n4zz63=-(n5zz61.*s5) + c5.*wp36;
	dg4zz6=n4zz63;
	n3zz61=c4.*n4zz61 - n5zz63.*s4;
	n3zz63=c4.*n5zz63 + n4zz61.*s4;
	dg3zz6=n3zz63;
	n2zz61=c3.*n3zz61 + n4zz63.*s3;
	n2zz62=-(c3.*n4zz63) + n3zz61.*s3;
	dg2zz6=n3zz63;
	n1zz63=-(c2.*n2zz62) - n2zz61.*s2;
	dg1zz6=n1zz63;
	dg6mx6=vp26;
	e5mx61=c6.*u116 - s6.*u216;
	e5mx63=s6.*u116 + c6.*u216;
	n5mx61=-(s6.*vp25);
	n5mx63=c6.*vp25;
	dg5mx6=n5mx63;
	e4mx61=c5.*e5mx61 + s5.*u316;
	e4mx63=-(e5mx61.*s5) + c5.*u316;
	n4mx61=c5.*n5mx61 + s5.*vp26;
	n4mx63=-(n5mx61.*s5) + c5.*vp26;
	dg4mx6=n4mx63;
	e3mx61=c4.*e4mx61 - e5mx63.*s4;
	e3mx63=c4.*e5mx63 + e4mx61.*s4;
	n3mx61=c4.*n4mx61 - n5mx63.*s4 - rl4.*(c4.*e5mx63 + e4mx61.*s4);
	n3mx63=c4.*n5mx63 + n4mx61.*s4 + rl4.*(c4.*e4mx61 - e5mx63.*s4);
	dg3mx6=n3mx63;
	n2mx61=c3.*n3mx61 + n4mx63.*s3 - rl3.*(-(c3.*e4mx63) + e3mx61.*s3);
	n2mx62=-(d3.*e3mx63) - c3.*n4mx63 + n3mx61.*s3 + rl3.*(c3.*e3mx61 + e4mx63.*s3);
	n2mx63=n3mx63 + d3.*(-(c3.*e4mx63) + e3mx61.*s3);
	dg2mx6=n2mx63;
	n1mx63=-(c2.*n2mx62) - n2mx61.*s2;
	dg1mx6=n1mx63;
	dg6my6=-vp16;
	e5my61=c6.*u126 - s6.*u226;
	e5my63=s6.*u126 + c6.*u226;
	n5my61=-(c6.*vp25);
	n5my63=-(s6.*vp25);
	dg5my6=n5my63;
	e4my61=c5.*e5my61 + s5.*u326;
	e4my63=-(e5my61.*s5) + c5.*u326;
	n4my61=c5.*n5my61 - s5.*vp16;
	n4my63=-(n5my61.*s5) - c5.*vp16;
	dg4my6=n4my63;
	e3my61=c4.*e4my61 - e5my63.*s4;
	e3my63=c4.*e5my63 + e4my61.*s4;
	n3my61=c4.*n4my61 - n5my63.*s4 - rl4.*(c4.*e5my63 + e4my61.*s4);
	n3my63=c4.*n5my63 + n4my61.*s4 + rl4.*(c4.*e4my61 - e5my63.*s4);
	dg3my6=n3my63;
	n2my61=c3.*n3my61 + n4my63.*s3 - rl3.*(-(c3.*e4my63) + e3my61.*s3);
	n2my62=-(d3.*e3my63) - c3.*n4my63 + n3my61.*s3 + rl3.*(c3.*e3my61 + e4my63.*s3);
	n2my63=n3my63 + d3.*(-(c3.*e4my63) + e3my61.*s3);
	dg2my6=n2my63;
	n1my63=-(c2.*n2my62) - n2my61.*s2;
	dg1my6=n1my63;
	e5mz61=c6.*u136 - s6.*u236;
	e5mz63=s6.*u136 + c6.*u236;
	n5mz61=-(s6.*vp16) - c6.*vp26;
	n5mz63=c6.*vp16 - s6.*vp26;
	dg5mz6=n5mz63;
	e4mz61=c5.*e5mz61 + s5.*u336;
	e4mz63=-(e5mz61.*s5) + c5.*u336;
	n4mz61=c5.*n5mz61;
	n4mz63=-(n5mz61.*s5);
	dg4mz6=n4mz63;
	e3mz61=c4.*e4mz61 - e5mz63.*s4;
	e3mz63=c4.*e5mz63 + e4mz61.*s4;
	n3mz61=c4.*n4mz61 - n5mz63.*s4 - rl4.*(c4.*e5mz63 + e4mz61.*s4);
	n3mz63=c4.*n5mz63 + n4mz61.*s4 + rl4.*(c4.*e4mz61 - e5mz63.*s4);
	dg3mz6=n3mz63;
	n2mz61=c3.*n3mz61 + n4mz63.*s3 - rl3.*(-(c3.*e4mz63) + e3mz61.*s3);
	n2mz62=-(d3.*e3mz63) - c3.*n4mz63 + n3mz61.*s3 + rl3.*(c3.*e3mz61 + e4mz63.*s3);
	n2mz63=n3mz63 + d3.*(-(c3.*e4mz63) + e3mz61.*s3);
	dg2mz6=n2mz63;
	n1mz63=-(c2.*n2mz62) - n2mz61.*s2;
	dg1mz6=n1mz63;
	e5m61=c6.*vp16 - s6.*vp26;
	e5m63=s6.*vp16 + c6.*vp26;
	e4m61=c5.*e5m61 - s5.*vp25;
	e4m63=-(e5m61.*s5) - c5.*vp25;
	e3m61=c4.*e4m61 - e5m63.*s4;
	e3m63=c4.*e5m63 + e4m61.*s4;
	n3m61=-(rl4.*(c4.*e5m63 + e4m61.*s4));
	n3m63=rl4.*(c4.*e4m61 - e5m63.*s4);
	dg3m6=n3m63;
	n2m61=c3.*n3m61 - rl3.*(-(c3.*e4m63) + e3m61.*s3);
	n2m62=-(d3.*e3m63) + n3m61.*s3 + rl3.*(c3.*e3m61 + e4m63.*s3);
	n2m63=n3m63 + d3.*(-(c3.*e4m63) + e3m61.*s3);
	dg2m6=n2m63;
	n1m63=-(c2.*n2m62) - n2m61.*s2;
	dg1m6=n1m63;
	dg6ia6=qdd6;
	dg6fv6=qd6;
	dg6fs6=sign(qd6);
	dg6xx7=-dv127;
	n5xx71=-(dv137.*s6) + c6.*wp16;
	n5xx73=c6.*dv137 + s6.*wp16;
	dg5xx7=n5xx73;
	n4xx71=c5.*n5xx71 - dv127.*s5;
	n4xx73=-(c5.*dv127) - n5xx71.*s5;
	dg4xx7=n4xx73;
	n3xx71=c4.*n4xx71 - n5xx73.*s4;
	n3xx73=c4.*n5xx73 + n4xx71.*s4;
	dg3xx7=n3xx73;
	n2xx71=c3.*n3xx71 + n4xx73.*s3;
	n2xx72=-(c3.*n4xx73) + n3xx71.*s3;
	dg2xx7=n3xx73;
	n1xx73=-(c2.*n2xx72) - n2xx71.*s2;
	dg1xx7=n1xx73;
	no7xy73=-dv117 + dv227;
	dg6xy7=no7xy73;
	n5xy71=-(c6.*u317) - s6.*u327;
	n5xy73=-(s6.*u317) + c6.*u327;
	dg5xy7=n5xy73;
	n4xy71=c5.*n5xy71 + no7xy73.*s5;
	n4xy73=c5.*no7xy73 - n5xy71.*s5;
	dg4xy7=n4xy73;
	n3xy71=c4.*n4xy71 - n5xy73.*s4;
	n3xy73=c4.*n5xy73 + n4xy71.*s4;
	dg3xy7=n3xy73;
	n2xy71=c3.*n3xy71 + n4xy73.*s3;
	n2xy72=-(c3.*n4xy73) + n3xy71.*s3;
	dg2xy7=n3xy73;
	n1xy73=-(c2.*n2xy72) - n2xy71.*s2;
	dg1xy7=n1xy73;
	no7xz72=dv117 - dv337;
	dg6xz7=-u237;
	n5xz71=-(no7xz72.*s6) + c6.*u217;
	n5xz73=c6.*no7xz72 + s6.*u217;
	dg5xz7=n5xz73;
	n4xz71=c5.*n5xz71 - s5.*u237;
	n4xz73=-(n5xz71.*s5) - c5.*u237;
	dg4xz7=n4xz73;
	n3xz71=c4.*n4xz71 - n5xz73.*s4;
	n3xz73=c4.*n5xz73 + n4xz71.*s4;
	dg3xz7=n3xz73;
	n2xz71=c3.*n3xz71 + n4xz73.*s3;
	n2xz72=-(c3.*n4xz73) + n3xz71.*s3;
	dg2xz7=n3xz73;
	n1xz73=-(c2.*n2xz72) - n2xz71.*s2;
	dg1xz7=n1xz73;
	dg6yy7=dv127;
	n5yy71=-(c6.*dv237) - s6.*wp26;
	n5yy73=-(dv237.*s6) + c6.*wp26;
	dg5yy7=n5yy73;
	n4yy71=c5.*n5yy71 + dv127.*s5;
	n4yy73=c5.*dv127 - n5yy71.*s5;
	dg4yy7=n4yy73;
	n3yy71=c4.*n4yy71 - n5yy73.*s4;
	n3yy73=c4.*n5yy73 + n4yy71.*s4;
	dg3yy7=n3yy73;
	n2yy71=c3.*n3yy71 + n4yy73.*s3;
	n2yy72=-(c3.*n4yy73) + n3yy71.*s3;
	dg2yy7=n3yy73;
	n1yy73=-(c2.*n2yy72) - n2yy71.*s2;
	dg1yy7=n1yy73;
	no7yz71=-dv227 + dv337;
	dg6yz7=u137;
	n5yz71=c6.*no7yz71 + s6.*u127;
	n5yz73=no7yz71.*s6 - c6.*u127;
	dg5yz7=n5yz73;
	n4yz71=c5.*n5yz71 + s5.*u137;
	n4yz73=-(n5yz71.*s5) + c5.*u137;
	dg4yz7=n4yz73;
	n3yz71=c4.*n4yz71 - n5yz73.*s4;
	n3yz73=c4.*n5yz73 + n4yz71.*s4;
	dg3yz7=n3yz73;
	n2yz71=c3.*n3yz71 + n4yz73.*s3;
	n2yz72=-(c3.*n4yz73) + n3yz71.*s3;
	dg2yz7=n3yz73;
	n1yz73=-(c2.*n2yz72) - n2yz71.*s2;
	dg1yz7=n1yz73;
	dg6zz7=wp36;
	n5zz71=c6.*dv237 + dv137.*s6;
	n5zz73=-(c6.*dv137) + dv237.*s6;
	dg5zz7=n5zz73;
	n4zz71=c5.*n5zz71 + s5.*wp36;
	n4zz73=-(n5zz71.*s5) + c5.*wp36;
	dg4zz7=n4zz73;
	n3zz71=c4.*n4zz71 - n5zz73.*s4;
	n3zz73=c4.*n5zz73 + n4zz71.*s4;
	dg3zz7=n3zz73;
	n2zz71=c3.*n3zz71 + n4zz73.*s3;
	n2zz72=-(c3.*n4zz73) + n3zz71.*s3;
	dg2zz7=n3zz73;
	n1zz73=-(c2.*n2zz72) - n2zz71.*s2;
	dg1zz7=n1zz73;
	dg6mx7=vp26;
	e5mx71=c6.*u117 - s6.*u217;
	e5mx73=s6.*u117 + c6.*u217;
	n5mx71=-(s6.*vp25);
	n5mx73=c6.*vp25;
	dg5mx7=n5mx73;
	e4mx71=c5.*e5mx71 + s5.*u317;
	e4mx73=-(e5mx71.*s5) + c5.*u317;
	n4mx71=c5.*n5mx71 + s5.*vp26;
	n4mx73=-(n5mx71.*s5) + c5.*vp26;
	dg4mx7=n4mx73;
	e3mx71=c4.*e4mx71 - e5mx73.*s4;
	e3mx73=c4.*e5mx73 + e4mx71.*s4;
	n3mx71=c4.*n4mx71 - n5mx73.*s4 - rl4.*(c4.*e5mx73 + e4mx71.*s4);
	n3mx73=c4.*n5mx73 + n4mx71.*s4 + rl4.*(c4.*e4mx71 - e5mx73.*s4);
	dg3mx7=n3mx73;
	n2mx71=c3.*n3mx71 + n4mx73.*s3 - rl3.*(-(c3.*e4mx73) + e3mx71.*s3);
	n2mx72=-(d3.*e3mx73) - c3.*n4mx73 + n3mx71.*s3 + rl3.*(c3.*e3mx71 + e4mx73.*s3);
	n2mx73=n3mx73 + d3.*(-(c3.*e4mx73) + e3mx71.*s3);
	dg2mx7=n2mx73;
	n1mx73=-(c2.*n2mx72) - n2mx71.*s2;
	dg1mx7=n1mx73;
	dg6my7=-vp16;
	e5my71=c6.*u127 - s6.*u227;
	e5my73=s6.*u127 + c6.*u227;
	n5my71=-(c6.*vp25);
	n5my73=-(s6.*vp25);
	dg5my7=n5my73;
	e4my71=c5.*e5my71 + s5.*u327;
	e4my73=-(e5my71.*s5) + c5.*u327;
	n4my71=c5.*n5my71 - s5.*vp16;
	n4my73=-(n5my71.*s5) - c5.*vp16;
	dg4my7=n4my73;
	e3my71=c4.*e4my71 - e5my73.*s4;
	e3my73=c4.*e5my73 + e4my71.*s4;
	n3my71=c4.*n4my71 - n5my73.*s4 - rl4.*(c4.*e5my73 + e4my71.*s4);
	n3my73=c4.*n5my73 + n4my71.*s4 + rl4.*(c4.*e4my71 - e5my73.*s4);
	dg3my7=n3my73;
	n2my71=c3.*n3my71 + n4my73.*s3 - rl3.*(-(c3.*e4my73) + e3my71.*s3);
	n2my72=-(d3.*e3my73) - c3.*n4my73 + n3my71.*s3 + rl3.*(c3.*e3my71 + e4my73.*s3);
	n2my73=n3my73 + d3.*(-(c3.*e4my73) + e3my71.*s3);
	dg2my7=n2my73;
	n1my73=-(c2.*n2my72) - n2my71.*s2;
	dg1my7=n1my73;
	e5mz71=c6.*u137 - s6.*u237;
	e5mz73=s6.*u137 + c6.*u237;
	n5mz71=-(s6.*vp16) - c6.*vp26;
	n5mz73=c6.*vp16 - s6.*vp26;
	dg5mz7=n5mz73;
	e4mz71=c5.*e5mz71 + s5.*u337;
	e4mz73=-(e5mz71.*s5) + c5.*u337;
	n4mz71=c5.*n5mz71;
	n4mz73=-(n5mz71.*s5);
	dg4mz7=n4mz73;
	e3mz71=c4.*e4mz71 - e5mz73.*s4;
	e3mz73=c4.*e5mz73 + e4mz71.*s4;
	n3mz71=c4.*n4mz71 - n5mz73.*s4 - rl4.*(c4.*e5mz73 + e4mz71.*s4);
	n3mz73=c4.*n5mz73 + n4mz71.*s4 + rl4.*(c4.*e4mz71 - e5mz73.*s4);
	dg3mz7=n3mz73;
	n2mz71=c3.*n3mz71 + n4mz73.*s3 - rl3.*(-(c3.*e4mz73) + e3mz71.*s3);
	n2mz72=-(d3.*e3mz73) - c3.*n4mz73 + n3mz71.*s3 + rl3.*(c3.*e3mz71 + e4mz73.*s3);
	n2mz73=n3mz73 + d3.*(-(c3.*e4mz73) + e3mz71.*s3);
	dg2mz7=n2mz73;
	n1mz73=-(c2.*n2mz72) - n2mz71.*s2;
	dg1mz7=n1mz73;
	e5m71=c6.*vp16 - s6.*vp26;
	e5m73=s6.*vp16 + c6.*vp26;
	e4m71=c5.*e5m71 - s5.*vp25;
	e4m73=-(e5m71.*s5) - c5.*vp25;
	e3m71=c4.*e4m71 - e5m73.*s4;
	e3m73=c4.*e5m73 + e4m71.*s4;
	n3m71=-(rl4.*(c4.*e5m73 + e4m71.*s4));
	n3m73=rl4.*(c4.*e4m71 - e5m73.*s4);
	dg3m7=n3m73;
	n2m71=c3.*n3m71 - rl3.*(-(c3.*e4m73) + e3m71.*s3);
	n2m72=-(d3.*e3m73) + n3m71.*s3 + rl3.*(c3.*e3m71 + e4m73.*s3);
	n2m73=n3m73 + d3.*(-(c3.*e4m73) + e3m71.*s3);
	dg2m7=n2m73;
	n1m73=-(c2.*n2m72) - n2m71.*s2;
	dg1m7=n1m73;


% *=*
% number of operations : 666 '+' or '-', 1147 '*' or '/'

% Fin du modèle SYMORO

%% Nettoyage des variables intermédiaires pour le modéle d'identification
% - générer un fichier tx40_dim.c par SYMORO avec l'option Optimizer/output C
% - copier en remplacant "double" par "clear", et les "," et ";" par espace

% à mettre en commentaire pour gagner du temps
% pour le calcul de idm pour la simulation (idm*x)

clear s2  c2  s3  c3  s4  c4  s5  c5  s6  c6 
clear wi12  wi22  wp12  wp22  dv112  dv222  dv332  dv122  dv132  dv232 
clear u112  u122  u132  u212  u232  u312  u322  u332  vp12  vp22 
clear wi13  wi23  w33  wp13  wp23  wp33  dv113  dv223  dv333  dv123 
clear dv133  dv233  u113  u123  u133  u213  u223  u233  u313  u323 
clear u333  vsp13  vsp23  vsp33  vp13  vp23  wi14  wi24  w34  wp14 
clear wp24  wp34  dv114  dv224  dv334  dv124  dv134  dv234  u114  u124 
clear u134  u214  u224  u234  u314  u324  u334  vsp14  vsp24  vsp34 
clear vp14  vp24  wi15  wi25  w35  wp15  wp25  wp35  dv115  dv225 
clear dv335  dv125  dv135  dv235  u115  u125  u135  u215  u225  u235 
clear u315  u325  u335  vp15  vp25  wi16  wi26  w36  wp16  wp26 
clear wp36  dv116  dv226  dv336  dv126  dv136  dv236  u116  u126  u136 
clear u216  u226  u236  u316  u326  u336  vp16  vp26  dv117  dv227 
clear dv337  dv127  dv137  dv237  u117  u127  u137  u217  u227  u237 
clear u317  u327  u337  n1xx23  no2xy23  n1xy23  no2xz22  n1xz23  n1yy23  no2yz21 
clear n1yz23  n1zz23  n1mz23  n2xx31  n2xx32  n1xx33  no3xy33  n2xy31  n2xy32  n1xy33 
clear no3xz32  n2xz31  n2xz32  n1xz33  n2yy31  n2yy32  n1yy33  no3yz31  n2yz31  n2yz32 
clear n1yz33  n2zz31  n2zz32  n1zz33  n2mx31  n2mx32  n2mx33  n1mx33  n2my31  n2my32 
clear n2my33  n1my33  n2mz31  n2mz32  n2mz33  n1mz33  n2m31  n2m32  n2m33  n1m33 
clear n3xx41  n3xx43  n2xx41  n2xx42  n1xx43  no4xy43  n3xy41  n3xy43  n2xy41  n2xy42 
clear n1xy43  no4xz42  n3xz41  n3xz43  n2xz41  n2xz42  n1xz43  n3yy41  n3yy43  n2yy41 
clear n2yy42  n1yy43  no4yz41  n3yz41  n3yz43  n2yz41  n2yz42  n1yz43  n3zz41  n3zz43 
clear n2zz41  n2zz42  n1zz43  e3mx41  e3mx43  n3mx41  n3mx43  n2mx41  n2mx42  n2mx43 
clear n1mx43  e3my41  e3my43  n3my41  n3my43  n2my41  n2my42  n2my43  n1my43  e3mz41 
clear e3mz43  n3mz41  n3mz43  n2mz41  n2mz42  n2mz43  n1mz43  e3m41  e3m43  n3m41 
clear n3m43  n2m41  n2m42  n2m43  n1m43  n4xx51  n4xx53  n3xx51  n3xx53  n2xx51 
clear n2xx52  n1xx53  no5xy53  n4xy51  n4xy53  n3xy51  n3xy53  n2xy51  n2xy52  n1xy53 
clear no5xz52  n4xz51  n4xz53  n3xz51  n3xz53  n2xz51  n2xz52  n1xz53  n4yy51  n4yy53 
clear n3yy51  n3yy53  n2yy51  n2yy52  n1yy53  no5yz51  n4yz51  n4yz53  n3yz51  n3yz53 
clear n2yz51  n2yz52  n1yz53  n4zz51  n4zz53  n3zz51  n3zz53  n2zz51  n2zz52  n1zz53 
clear e4mx51  e4mx53  n4mx51  n4mx53  e3mx51  e3mx53  n3mx51  n3mx53  n2mx51  n2mx52 
clear n2mx53  n1mx53  e4my51  e4my53  n4my51  n4my53  e3my51  e3my53  n3my51  n3my53 
clear n2my51  n2my52  n2my53  n1my53  e4mz51  e4mz53  n4mz51  n4mz53  e3mz51  e3mz53 
clear n3mz51  n3mz53  n2mz51  n2mz52  n2mz53  n1mz53  e4m51  e4m53  e3m51  e3m53 
clear n3m51  n3m53  n2m51  n2m52  n2m53  n1m53  n5xx61  n5xx63  n4xx61  n4xx63 
clear n3xx61  n3xx63  n2xx61  n2xx62  n1xx63  no6xy63  n5xy61  n5xy63  n4xy61  n4xy63 
clear n3xy61  n3xy63  n2xy61  n2xy62  n1xy63  no6xz62  n5xz61  n5xz63  n4xz61  n4xz63 
clear n3xz61  n3xz63  n2xz61  n2xz62  n1xz63  n5yy61  n5yy63  n4yy61  n4yy63  n3yy61 
clear n3yy63  n2yy61  n2yy62  n1yy63  no6yz61  n5yz61  n5yz63  n4yz61  n4yz63  n3yz61 
clear n3yz63  n2yz61  n2yz62  n1yz63  n5zz61  n5zz63  n4zz61  n4zz63  n3zz61  n3zz63 
clear n2zz61  n2zz62  n1zz63  e5mx61  e5mx63  n5mx61  n5mx63  e4mx61  e4mx63  n4mx61 
clear n4mx63  e3mx61  e3mx63  n3mx61  n3mx63  n2mx61  n2mx62  n2mx63  n1mx63  e5my61 
clear e5my63  n5my61  n5my63  e4my61  e4my63  n4my61  n4my63  e3my61  e3my63  n3my61 
clear n3my63  n2my61  n2my62  n2my63  n1my63  e5mz61  e5mz63  n5mz61  n5mz63  e4mz61 
clear e4mz63  n4mz61  n4mz63  e3mz61  e3mz63  n3mz61  n3mz63  n2mz61  n2mz62  n2mz63 
clear n1mz63  e5m61  e5m63  e4m61  e4m63  e3m61  e3m63  n3m61  n3m63  n2m61 
clear n2m62  n2m63  n1m63  n5xx71  n5xx73  n4xx71  n4xx73  n3xx71  n3xx73  n2xx71 
clear n2xx72  n1xx73  no7xy73  n5xy71  n5xy73  n4xy71  n4xy73  n3xy71  n3xy73  n2xy71 
clear n2xy72  n1xy73  no7xz72  n5xz71  n5xz73  n4xz71  n4xz73  n3xz71  n3xz73  n2xz71 
clear n2xz72  n1xz73  n5yy71  n5yy73  n4yy71  n4yy73  n3yy71  n3yy73  n2yy71  n2yy72 
clear n1yy73  no7yz71  n5yz71  n5yz73  n4yz71  n4yz73  n3yz71  n3yz73  n2yz71  n2yz72 
clear n1yz73  n5zz71  n5zz73  n4zz71  n4zz73  n3zz71  n3zz73  n2zz71  n2zz72  n1zz73 
clear e5mx71  e5mx73  n5mx71  n5mx73  e4mx71  e4mx73  n4mx71  n4mx73  e3mx71  e3mx73 
clear n3mx71  n3mx73  n2mx71  n2mx72  n2mx73  n1mx73  e5my71  e5my73  n5my71  n5my73 
clear e4my71  e4my73  n4my71  n4my73  e3my71  e3my73  n3my71  n3my73  n2my71  n2my72 
clear n2my73  n1my73  e5mz71  e5mz73  n5mz71  n5mz73  e4mz71  e4mz73  n4mz71  n4mz73 
clear e3mz71  e3mz73  n3mz71  n3mz73  n2mz71  n2mz72  n2mz73  n1mz73  e5m71  e5m73 
clear e4m71  e4m73  e3m71  e3m73  n3m71  n3m73  n2m71  n2m72  n2m73  n1m73


%% Prise en compte du couplage entre l'axe 5 et l'axe 6
%%les couples enregitrés sont les couples moteurs
%%ramenés coté articulaire avec J'
%%fournis par le controleur staubli CS8C

% % D'après l'article icra 2007 :
% 
% % Axe 5
% dg5ia6 = qdd6;
% dg5fvm6 = qd6;
% dg5fsm6 = sign(qd6);
% % Axe 6
% dg6ia6 = dg6ia6 + qdd5;
% dg6fvm6 = qd5;
% dg6fsm6= sign(qd5);
% 
% % fin icra 2007

% % D'après l'article ifac_wc 2011 :
% 
% % Axe 5
% dg5ia6 = qdd6;
% dg5fvm6 = qd6;
% %dg5fsm6 = sign(qd6);
% dg5fsm6 = sign(qd5+qd6)-sign(qd5);%pour symétrie
% % Axe 6
% dg6ia6 = dg6ia6 + qdd5;
% dg6fvm6 = qd5;
% dg6fsm6= sign(qd5+qd6)-sign(qd6);
% 
% % fin ifac_wc 2011

% % D'après khalil:
% 
% % Axe 5
% dg5ia6 = qdd6;
% dg5fvm6 = qd6;
% dg5fsm6 = sign(qd5+qd6);
% % Axe 6
% dg6ia6 = dg6ia6 + qdd5;
% dg6fvm6 = qd5;
% dg6fsm6= sign(qd5+qd6);
% 
% % fin khalil

% D'après gautier ieeeasmetmech:
% couplage symétrique

% Axe 5
dg5iam6 = qdd6;
dg5fvm6 = qd6;
dg5fsm6 = sign(qd5+qd6);
% Axe 6
dg6iam6 = qdd5;
dg6fvm6 = qd5;
dg6fsm6= sign(qd5+qd6);

% fin gautier ieeesmetmech

%fin du couplage entre l'axe 5 et l'axe 6

% Contribution des offsets

nm=size(q1);
dgoff=ones(nm);
for k=1:narta
    %     dgoff=ones(nm);
    eval(['dg' int2str(k) 'off' int2str(k) ' = dgoff ;']) ;
end
clear dgoff;

% Fin du programme tx40s_charge_dim.m
