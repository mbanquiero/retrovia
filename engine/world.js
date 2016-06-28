/**
 * @fileoverview world.js
<P> motor de fisica 2d de proposito didactivo basado en impulse resolution</P>
<P>  features</P>
<P>  - rigid body: circulos y poligonos concavos</P>
<P>  - soporta oriented rigid body </P>
<P>  - algoritmo de integracion: euler modificado</P>
<P> - algoritmo de deteccion de colisiones: SAT</P>
<P>  - constratints : joints </P>
<P>  - Ragdall y softbidy : implmementados con distance constraints</P>
*/

/**
 * World
 * @class World Escena
 * @property {float} time tiempo global
 * @property {int} m_bodyCount cantidad de objetos (Body)
 * @property {Body []} bodies array de cuerpos rigidos que componen la escena
 * @property {int} m_contactCount cantidad de contactos entre cuerpos
 * @property {ContactSolver []} contacts Array de contactos
 * @property {int} m_constraintCount cantidad de restricciones
 * @property {Constraint []} constraints Array de restricciones (constraints)
 * @constructor
 */
function World()
{
    // tiempo global
    this.time = 0;

    // cuerpos rigidos 
    this.m_bodyCount = 0;
    this.bodies = [];

    // Contactos entre los cuerpos rigidos
    this.m_contactCount = 0;
    this.contacts = [];

    // restricciones 
    this.m_constraintCount = 0;
    this.constraints = [];
}

/**
 * Genero informacion del contacto entre todos los cuerpos: Narrow Phase
 * @method Update
 * @param {float} elapsed_time tiempo transcurrido
*/
World.prototype.Update = function(elapsed_time)
{

    // Genero informacion del contacto entre todos los cuerpos: Narrow Phase
    this.m_contactCount = 0;
    for(var i = 0; i < this.m_bodyCount-1; ++i)
    {
        var A = this.bodies[i];

		// armo la lista de colliders, objetos con los que puede colisionar
		var collider_list;
		var cant_colliders = 0;
		if(A.collider_list!=null)
		{
			// si ya tiene una lista de collider, tomo esa lista
			collider_list = A.collider_list;
			cant_colliders = collider_list.length;
		}
		else
		{
			// si no, puede colisionar con cualquier objeto de la lista 
			collider_list = [];
			for(var j = i + 1; j < this.m_bodyCount; ++j)
				 collider_list[cant_colliders++] = this.bodies[j];
		}	
		
		for(var j = 0; j < cant_colliders; ++j)
        {
            var B = collider_list[j];
            // si ambos cuerpos son estaticos, no pueden colisionar
            if(A.inverseMass == 0 && B.inverseMass== 0)
                continue;

            // si Ambos cuerpos forman un joint tampoco pueden colisionar, aunque si lo pueden hacer en la vida real 
            // el solver no es estable con este tipo de configuracion 
            var C = new ContactSolver();
            if(C.BodyCollide(elapsed_time , A,B))
            {
                this.contacts[this.m_contactCount++] = C;
            }
        }

		/*
        for(var j = i + 1; j < this.m_bodyCount; ++j)
        {
            var B = this.bodies[j];
            // si ambos cuerpos son estaticos, no pueden colisionar
            if(A.inverseMass == 0 && B.inverseMass== 0)
                continue;

            // si Ambos cuerpos forman un joint tampoco pueden colisionar, aunque si lo pueden hacer en la vida real 
            // el solver no es estable con este tipo de configuracion 
            var C = new ContactSolver();
            if(C.BodyCollide(elapsed_time , A,B))
            {
                this.contacts[this.m_contactCount++] = C;
            }
        }*/
    }

    // Integrar fuerzas primer parte
    for(var i = 0; i < this.m_bodyCount; ++i)
        this.bodies[i].IntegrateForces( elapsed_time/2.);

    // Precalculo la info de contactos
    for(var i=0;i<this.m_contactCount;++i)
        this.contacts[i].PreCalc();

    // Fase de Collision Response: Impulse Resolution
    var cant_iteracciones = 100;
    for(var j = 0; j < cant_iteracciones; ++j)
    for(var i = 0; i < this.m_contactCount; ++i)
        this.contacts[i].ApplyImpulse();
		
    // Integrar velocidades
    for(var i = 0; i < this.m_bodyCount; ++i)
        this.bodies[i].IntegrateVelocity(  elapsed_time );

    // Integrar fuerzas segunda parte
    for(var i = 0; i < this.m_bodyCount; ++i)
        this.bodies[i].IntegrateForces( elapsed_time/2.);

    // Corregir posiciones (eliminar penetraciones)
    for(var i = 0; i < this.m_contactCount; ++i)
        this.contacts[i].positionCorrection( );

    // Limpiar todas las fuerzas, pues ya fueron integradas
    for(var i = 0; i < this.m_bodyCount; ++i)
    {
        this.bodies[i].force.Set(0,0);
        this.bodies[i].torque = 0;
    }

    // Solve constrains
    for(var j = 0; j < 3; ++j)
        for(var i = 0; i < this.m_constraintCount; ++i)
            this.constraints[i].Solve();


    // saco de la simulacion los objetos borados
    for (var i = 0; i < this.m_bodyCount; ++i) {
        var A = this.bodies[i];
        if (A.deleted) {
            A.position.Set(0, 0);
            A.SetStatic();
        }
    }


    // avanzo el tiempo global
    this.time += elapsed_time;

}


/*
World.prototype.ComputeWeight = function (A)
{
    // busco en la lista de contactos, todos los cuerpos que estan en contacto con A
    var weight = A.mass;
    for (var i = 0; i < this.m_contactCount; ++i)
    {
        var contact = this.contacts[i];
        var bodyB = null;
        if (contact.A.id == A.id)
        {
            // el cuerpo B del contacto hace contacto con A
            bodyB = contact.B;
        }
        else
        if (contact.B.id == A.id)
        {
            // el cuerpo A del contacto hace contacto con A
            bodyB = contact.A;
        }

        if(bodyB!=null)
        {
            // verifico que el cuerpo B este "arriba" del A 
            if(bodyB.position.y < A.position.y)
            {
                // supongo que el cuerpo B esta apoyado
                weight+=bodyB.mass;
            }
        }
    }
        
    return weight;
}
*/

/**
 * @method Render
*/
World.prototype.Render = function()
{
    // Dibujo los cuerpos
    ctx.font = "12px Arial";
    for(var i = 0; i < this.m_bodyCount; ++i)
    {
        var A = this.bodies[i];
        A.Render();

        /*
        // debug max impulse
        if (A.inverseMass != 0 && A.maxImpulse!=0)
        {
            //var W = _world.ComputeWeight(A);
            //ctx.fillText("" + W.toFixed(1), ox + A.position.x * ex, oy + A.position.y * ey-20);
            ctx.fillText("" + A.maxImpulse.toFixed(1), ox + A.position.x * ex, oy + A.position.y * ey-20);
        }
        */
    }

    // Dibujo los contactos
    ctx.fillStyle = '#FFFFFF';
    ctx.strokeStyle = '#3030FF';
    for(var i = 0; i < this.m_contactCount; ++i)
    {
        for(var j = 0; j < this.contacts[i].contact_count; ++j)
        {
            var c = this.contacts[i].contacts[j];
            var n = this.contacts[i].normal;

            // dibujo un circulo en la posicion de contacto
            ctx.beginPath();
            ctx.arc(ox + c.x*ex, oy + c.y*ey, 5, 0, Math.PI*2,true); 
            ctx.closePath();
            ctx.fill();
            ctx.stroke();

            // dibujo una linea indicando la direccion normal
            ctx.beginPath();
            ctx.moveTo(ox + c.x*ex, oy + c.y*ey);
            ctx.lineTo(ox + c.x *ex + n.x*20, oy + c.y*ey+n.y*20);
            ctx.stroke();
        }
    }


    // dibujo los constraints
    ctx.strokeStyle = '#F0F0FF';
    for(var i = 0; i < this.m_constraintCount; ++i)
    {
        switch(this.constraints[i].visible)
        {
            case 1:
                // dibujo una linea llena fina 
                ctx.beginPath();
                var c = vec2_transform(this.constraints[i].A.shape.matWorld, this.constraints[i].ptA);
                c.add(this.constraints[i].A.position);
                ctx.moveTo(ox + c.x*ex ,oy + c.y*ey);

                c = vec2_transform(this.constraints[i].B.shape.matWorld , this.constraints[i].ptB);
                c.add(this.constraints[i].B.position);
                ctx.lineTo(ox + c.x * ex, oy + c.y * ey);
                ctx.stroke();
                break;

            case 2:
                // dibujo una linea gruesa (usualmente son joints y quiero dibujar el hueso)
                var p0 = vec2_transform(this.constraints[i].A.shape.matWorld, this.constraints[i].ptA);
                p0.add(this.constraints[i].A.position);
                var p1 = vec2_transform(this.constraints[i].B.shape.matWorld, this.constraints[i].ptB);
                p1.add(this.constraints[i].B.position);
                var dir_v = vec2_substract(p1 , p0);
                dir_v.normalize();
                var dir_w = vec2_normal(dir_v);
                var rA = this.constraints[i].A.shape.radius;
                var rB = this.constraints[i].B.shape.radius;
                var Q = [];
                Q[0] = vec2_add ( p0  , vec2_multiply(dir_w , rA));
                Q[1] = vec2_add ( p1  , vec2_multiply(dir_w , rB));
                Q[2] = vec2_substract ( p1  , vec2_multiply(dir_w , rB));
                Q[3] = vec2_substract(p0, vec2_multiply(dir_w, rA));

                ctx.fillStyle = '#f0f0f0';
                ctx.beginPath();
                ctx.moveTo(ox + Q[0].x * ex, oy + Q[0].y * ey);
                for(var t=1;t<4;++t)
                    ctx.lineTo(ox + Q[t].x * ex, oy + Q[t].y * ey);
                ctx.closePath();
                ctx.fill();
                ctx.stroke();
                break;
        }
    }
}

/**
 * Limpia toda la escena. 
 * @method Clear
*/
World.prototype.Clear = function()
{
    this.m_bodyCount = 0;
    this.m_contactCount = 0;
    this.m_constraintCount = 0;
}

// interface de creacion de objetos

/**
 * Agrega un cuerpo rigido con forma de circulo
 * @method AddCircle
 * @param {float} x posicion x
 * @param {float} y posicion y
 * @param {float} r radio
*/
World.prototype.AddCircle = function( x, y , r)
{
    var b = this.bodies[this.m_bodyCount] = new Body();
    b.CreateCircle(r, new Vector2(x, y), 1);
    b.id = this.m_bodyCount++;
    return b;
}

/**
 * Agrega un cuerpo rigido con forma de caja rectangular
 * @method AddBox
 * @param {float} x posicion x
 * @param {float} y posicion y
 * @param {float} w width
 * @param {float} h height
*/
World.prototype.AddBox = function( x, y , w,h)
{
    var b = this.bodies[this.m_bodyCount] = new Body();
    b.CreateBox(w,h,new Vector2(x,y),1);
    b.id = this.m_bodyCount++;
    return b;
}
 
/**
 * Agrega un cuerpo rigido con forma triangular
 * @method AddTri
 * @param {float} ax posicion x del punto a
 * @param {float} ay posicion y del punto a
 * @param {float} bx posicion x del punto b
 * @param {float} by posicion y del punto b
 * @param {float} cx posicion x del punto c
 * @param {float} cy posicion y del punto c
*/
World.prototype.AddTri = function( ax, ay , bx, by ,cx, cy )
{
	var v = [new Vector2(ax,ay) , new Vector2(bx,by) , new Vector2(cx,cy)];
    return this.AddPoly(v,3);
}
 
/**
 * Agrega un cuerpo rigido con forma de poligono convexo
 * @method AddPoly
 * @param {Vector2[]} pt array de puntos
 * @param {int} cant_pt cantidad de puntos
*/
World.prototype.AddPoly = function( pt,cant_pt)
{
    var b = this.bodies[this.m_bodyCount] = new Body();
    b.CreatePoly(pt, cant_pt, 1);
    b.id = this.m_bodyCount++;
    return b;
}

// 
// Body *A,Body *B,float dist

/**
 * Agregar un constraint generico de distancia entre los cuerpos A y B.
 * @method AddConstraint
 * @param {Body} A body A
 * @param {Body} B body B
 * @param {float} dist distancia |A.pos - B.pos| == dist
*/
World.prototype.AddConstraint = function (A, B, dist) {

    return this.constraints[this.m_constraintCount++] = new Constraint(A, B, dist);
}

/**
 * Agregar un constraint distancia entre los cuerpos A y B.
 * @method AddDistanceConstraint
 * @param {Body} A body A
 * @param {Vector2} ptA pivote o punto de contacto con el cuerpo A
 * @param {Body} B body B
 * @param {Vector2} ptB pivote o punto de contacto con el cuerpo B
*/

World.prototype.AddDistanceConstraint = function(A,ptA , B,ptB)
{
    return this.constraints[this.m_constraintCount++] = CreateDistanceConstraint(A, ptA, B,ptB);
}

/**
 * Agregar un constraint del tipo junta o articulacion entre los cuerpos A y B.
 * @method AddJointConstraint
 * @param {Body} A body A
 * @param {Body} B body B
*/
World.prototype.AddJointConstraint = function (A, B) {

    var c = this.constraints[this.m_constraintCount++] = new CreateDistanceConstraint(A, new Vector2(0, 0), B, new Vector2(0, 0));
    c.joint = true;
    c.exact_distance = true;
    c.angle_constraint = false;
    c.visible = 2;
    return c;
}

/*

    Constraint *AddAngleConstraint(Body *A,Body *B,Body *C);
    bool is_joint(Body *A,Body *B);			// devuelve true si el cuerpo A y el B forman un joint
*/

/**
 * interaccion c/ usuario: devuelve si el pto esta cercano al centro de gravedad de un cuerpo
 * @method PointNearBodyCM
 * @param {Vector2} pt posicion del mouse 
 * @param {float} dE -delta epsilon de contacto
*/
World.prototype.PointNearBodyCM = function(pt, dE)
{
	var rta = -1;
    var min_dist = dE*dE;
    for(var i = 0;i<this.m_bodyCount;++i)
    if(this.bodies[i].inverseMass!=0)		// solo se aplica a objetos dinamicos
    {
        var dist = vec2_substract(pt, this.bodies[i].position).lengthSqr();
        if(dist<min_dist)
        {
            rta = i;
            min_dist = dist;
        }
    }
    return rta;
}



