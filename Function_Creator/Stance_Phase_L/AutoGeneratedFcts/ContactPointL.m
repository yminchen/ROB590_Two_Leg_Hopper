function ContactPointL = ContactPointL(x,y,thetaL,thetaR,g,lCG,lL,mL,jL)
%CONTACTPOINTL
%    CONTACTPOINTL = CONTACTPOINTL(X,Y,THETAL,THETAR,G,LCG,LL,ML,JL)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    08-Jan-2017 14:10:15

ContactPointL = [x+lL.*sin(thetaL);y-lL.*cos(thetaL)];