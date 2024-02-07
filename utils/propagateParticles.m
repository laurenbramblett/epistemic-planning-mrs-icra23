function particles = propagateParticles(particles,forceX,forceY)
    uk = [forceX,forceY];
    if sqrt(sum(uk.^2))>particles.vel
        uk = uk*particles.vel/sqrt(sum(uk.^2));
    end  
    headingPart = wrapToPi(atan2(uk(2),uk(1)) - particles.pose(3));
    kp = 0.8; headingPart = sign(headingPart)*min(abs(headingPart),pi/4)*kp;
    velAtt = particles.vel*abs(cos(headingPart));
    %Move all particles according to their heading
    
    particles.pose(1) = particles.pose(1) + velAtt*cos(particles.pose(3)).*particles.dt;
    particles.pose(2) = particles.pose(2) + velAtt*sin(particles.pose(3)).*particles.dt;    
    particles.pose(3) = wrapToPi(particles.pose(3) + headingPart);
end