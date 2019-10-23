using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.IO;



public class BulletMove : PoolObject {

	public enum TypeOfImmitation
	{
		Simple = 0,
		Advanced = 1,
		AxlesProjection  = 4,
		HeunsMethod = 2,
		NewAlgoritm = 5,
        NewAlgoritm2 =6,
        rungeKut = 3,
    }

	/// <summary>
	/// The type of bullet.
	/// </summary>
	public TypeOfImmitation TypeOfBullet;

	[Header ("General")]
	/// <summary>
	/// The speed.
	/// </summary>
	public int speed = 100; // начальная скорости снаряда
	/// <summary>
	/// The time to destroy.
	/// </summary>
	public float timeToDestroy = 5f; // время, через которое снаряд удалится 

	[Header ("Penetration")]
	/// <summary>
	/// The bullet material density.
	/// </summary>
	public float BulletMaterialDensity = 11.2f;
	/// <summary>
	/// The penetrate D angle.
	/// </summary>
	public float PenetrateDAngle = 0.1f;

	[Header ("Ricoshet")]
	/// <summary>
	/// The minimum ricochet chance.
	/// </summary>
	public int minRicochetChance;
	/// <summary>
	/// The ricoshet D angle.
	/// </summary>
	public float RicoshetDAngle = 0.2f;

	[Header ("Ballistic")]
	/// <summary>
	/// The ballistic factor.
	/// </summary>
	public float BallisticFactor; // баллиситический коэффициент снаряда
	[Range(0.0001f,10f)]
	/// <summary>
	/// The mass.
	/// </summary>
	public float mass = 0.0084f; // масса снаряда (кг)
	/// <summary>
	/// The zatyad mass.
	/// </summary>
	public float zatyadMass = 0.0032f; // масса порохового заряда
	/// <summary>
	/// The calibr.
	/// </summary>
	public float calibr = 0.00762f; // калибр снаряда (м)
	/// <summary>
	/// The i.
	/// </summary>
	public float i = 0.56f; // коэффициент формы снаряда

	[Space(10)]
	//========================
	float Ro = 1.29f;
	/// <summary>
	/// The wind velo.
	/// </summary>
	public Vector3 windVelo; 	// скорость и направление ветра (x,y,z) М/С
	private float dt; 			// время обновления

	private Vector3 gravity;	// сила тяжести (x,y,z)
	private Vector3 CurVelocity;// текущая скорость пули  (x,y,z)
	private Transform myTrans;	// переменная, хранящая позицию(x,y,z), углы поворота (x,y,z)
	private float timer; //  таймер для удаления объекта со сцены

	private Vector3 velocityDirection;	// направление полета (для поворота графического объекта снаряда в  правильном направлении)
	/// <summary>
	/// The velocity magnitude.
	/// </summary>
	public  float velocityMagnitude;	// текущая скорость пули 
	/// <summary>
	/// The E.
	/// </summary>
	public float EC; // кинетическая энергия снаряда 

	private bool movebul;
	private  float A; //Bullet cross section area A = pd2/4
	//=========================

	public bool debugTrajectory;
	//**************************
	private float _starrot; 	//только для лог файла
	private Vector3 _starrotVect; 	//только для лог файла
	private Vector3 _startpos; 	//только для лог файла
	/// <summary>
	/// The dec.
	/// </summary>
	public GameObject dec;
	//**************************

//	void Start () { // функция, запускающаяся один раз перед запуском сцены
//	}

	void Awake(){
		
//		if (BallisticFactor == 0) CalcBallisticFactor();	// если баллистический коэффициент не задали вручную, то рассчитываем
		gravity = Vector3.up * -9.81f; // инициализируем величину и напрвление силы тяжести
		Init ();
	}
		
	[ContextMenu("Initialise!")]
	void Init(){	// инициализация компонентов
		if (!myTrans) myTrans = transform;	
		CurVelocity = myTrans.forward * speed; // инициализируем начальную скорость в напралении куда смотрит пуля передом
		_startpos = myTrans.position;	//только для лог файла
		_starrotVect = myTrans.localEulerAngles;
		_starrot = 360f  - Mathf.Abs(myTrans.localEulerAngles.x);//только для лог файла
        O = -myTrans.localEulerAngles.x * Mathf.Deg2Rad;
        
		velocityMagnitude  = speed;

		A = Mathf.PI * calibr * calibr * 0.25f;
	}

	void FixedUpdate () { // функция обновления. срабатывает каждые 0.02 сек


		dt = Time.fixedDeltaTime; // время обновления ( 0.02 сек)
//    	debugSdvig();
		movebul = true;
		switch (TypeOfBullet) {
		case TypeOfImmitation.Simple:
			CalcVelocitySimple ();
			break;
		case TypeOfImmitation.Advanced:
			CalcVelocity (); // функция смещения пули за время dt
			break;
		case TypeOfImmitation.AxlesProjection:
			CalcVelocityAxles ();
                return;
			break;
		case TypeOfImmitation.HeunsMethod:
			CalcVelocityHeun ();  
			break;
		case TypeOfImmitation.NewAlgoritm:
			CalcVelocityNewAlg ();
			break;
        case TypeOfImmitation.NewAlgoritm2:
            CalcVelocityNewAlg2();
            break;
        case TypeOfImmitation.rungeKut:
            CalcVelocityRungeKutt();
            break;
        }
		CheckHits ();
		if (movebul)BulletMoveV ();	// функция сдвига пули на величину CurVelocity, рассчитанную в CalcVelocity()


		//... Уничтожение объекта снаряда ...// 
		timer += dt;
		if (myTrans.position.y < 0){ //   (myTrans.position.y < 0 || timer > timeToDestroy)
			ReturnToPool ();
		}

	}


// ********************* Calc Velocity Simple ***************************************************
	void CalcVelocitySimple(){			// рассчитываем скорость в следующий момент времени
		CurVelocity += gravity * dt;	// текущая скорость (м\с) + сила тяжести (м\с2) * dt(с)

		velocityDirection = CurVelocity.normalized;	// вектор направления движения
		myTrans.forward = velocityDirection;	// поворачиваем снаряд в направлении его движения
		velocityMagnitude = CurVelocity.magnitude;	// скорость снаряда (длина вектора  CurVelocity) 	
	}

// ********************* Calc Velocity New algoritm ***************************************************

	void CalcVelocityNewAlg(){
		acceleration = gravity;  // величина постоянного ускорения (учет гравитации) M/C^2

		velE = CurVelocity + dt * gravity - windVelo;										//	M/C 	направление снаряда без учета сопротивления воздуха
		acceleration -= velE.normalized * BulletDecelerationNewAlg(velE.magnitude);  		// M/C^2	вычисление сопротивления воздуха

		newVelocity =  dt * acceleration ;													// M/C		величина ускорения с учетом гравитации, направления и силы ветра, сопротивления воздуха 
		CurVelocity += newVelocity;															// M/C		новое смещение пули

		velocityDirection = CurVelocity.normalized;	// напрвление, куда "смотрит" пуля 						
		myTrans.forward = velocityDirection;		// поворачиваем снаряд передом по направлению полета 	
		velocityMagnitude = CurVelocity.magnitude;					// скорость снаряда (длина вектора  CurVelocity) 	

		EC = mass * velocityMagnitude * velocityMagnitude * 0.5f; 	// кинетическая энергия снаряда  			
	}

	float BulletDecelerationNewAlg(float speed){
		float b = Ro * 0.5f * A * BallisticFactor*0.1f * speed * speed;
		return b / mass;
	}

    // ********************* Calc Velocity New algoritm 2 **************************************************


    float O;

    void CalcVelocityNewAlg2(){
       
        velocityMagnitude += (-BallisticFactor * BulletDeceleration(velocityMagnitude) - 9.81f * Mathf.Sin(O))* dt;
  
        O += (-9.81f * Mathf.Cos(O) / velocityMagnitude) * dt * Mathf.Rad2Deg;

        CurVelocity = new Vector3(
                0,
                velocityMagnitude * Mathf.Sin(O) ,
                velocityMagnitude * Mathf.Cos(O) 
            );
    }

    // ********************* Calc Velocity Heun's Method ***************************************************

    void CalcVelocityHeun(){
		Vector3 currentPosition = transform.position;
		Vector3 currentVelocity = CurVelocity;

		Vector3 acceleartionFactorEuler = Physics.gravity;
		Vector3 acceleartionFactorHeun = Physics.gravity;

		Vector3 velocityFactor = currentVelocity;

        Vector3 pos_E = currentPosition + dt * velocityFactor;
		acceleartionFactorEuler +=  -currentVelocity.normalized *  BulletDeceleration(currentVelocity.magnitude);
		Vector3 vel_E = currentVelocity + dt * acceleartionFactorEuler;

		//Heuns method
		Vector3 pos_H = currentPosition + dt * 0.5f * (velocityFactor + vel_E);
		acceleartionFactorHeun +=  -vel_E.normalized *  BulletDeceleration(vel_E.magnitude);
		Vector3 vel_H = currentVelocity + dt * 0.5f * (acceleartionFactorEuler + acceleartionFactorHeun);

        //print("position = " + transform.position + "  Pos = " + pos_H);
        //print("Vel = " + vel_H + "  Predicted Pos = " + (transform.position + vel_H * dt));

        CurVelocity = vel_H;

		velocityDirection = CurVelocity.normalized;	// вектор направления движения
		myTrans.forward = velocityDirection;	// поворачиваем снаряд в направлении его движения
		velocityMagnitude = CurVelocity.magnitude;	// скорость снаряда (длина вектора  CurVelocity) 	
	}

// ********************* Calc Velocity Axles ***************************************************


	void CalcVelocityAxles(){
		Vector3 lr = myTrans.localEulerAngles;

		float O = (lr.x > 180) ? -(lr.x - 360f) : -lr.x;
		float N = (lr.y > 180) ? -(lr.y - 360f) : -lr.y;
		O *= Mathf.Deg2Rad;
		N *= Mathf.Deg2Rad;

		CurVelocity = new Vector3 (									// простая модель движения тела, брошенного под углом к горизонту,
			0,
			velocityMagnitude * Mathf.Sin(O) - 9.81f * dt ,
			velocityMagnitude * Mathf.Cos(O)
		);
			
		CurVelocity += new Vector3 (								// учет ветра
			windVelo.x * dt,
			windVelo.y * dt,
			windVelo.z * dt
		);

		float dec = BulletDeceleration (CurVelocity.magnitude); 	// ускорение сопротивления воздуха M/c2

		CurVelocity -= CurVelocity.normalized * dec * dt;			// учет сопротивления воздуха

		newVelocity = Matrix4x4.Rotate (Quaternion.Euler (new Vector3 (0, lr.y, 0))) * CurVelocity;			// поворачиваем пулю вокруг вертикальной оси в направлении выстрела



		velocityMagnitude = CurVelocity.magnitude;
		myTrans.forward = newVelocity.normalized;
		myTrans.localPosition += newVelocity * dt;
	}
// ************************************************************************
	public Vector3 GetLocalPosition(){
		Vector3 LocalPos = ( myTrans.position - _startpos);
		LocalPos = RotatePointAroundPivot (LocalPos, _startpos, -Vector3.up *  _starrotVect.y);
		return LocalPos;
	}

	Vector3 RotatePointAroundPivot(Vector3 point, Vector3 pivot, Vector3 angle ){
		Vector3 dir = point - pivot;
		dir = Quaternion.Euler(angle) * dir;
		point = dir + pivot;
		return point;
	}

	void debugSdvig(){
	//	Vector3 sdp = transform.InverseTransformDirection (myTrans.position - _startpos);
		Vector3 sdp = GetLocalPosition(); //( myTrans.position - _startpos)  ;

		Debug.Log ("x = " + sdp.x + "  y = " + sdp.y + "  z = " + sdp.z + "  E = " + EC + "  Velocity = " + velocityMagnitude + "  time = " + timer + "  dt = " + dt);
	}

    // ***********Clac Velocity Runge Kutt******************************************************

    float vel, x1, x2, x3, x4, y1, y2, y3, y4, z1, z2, z3, z4;
    void CalcVelocityRungeKutt() {

        /*
        vel = veloc(CurVelocity.x, CurVelocity.y, 0);   // m/s
        x1 = dt * Fvx(CurVelocity.x, vel);                          // s * m/s2 = m/s
        y1 = dt * Fvy(CurVelocity.y, vel);
//        z1 = dt * Fvz(CurVelocity.z, vel);
        //       print( " cur VEl = "+ CurVelocity+"vel = " + vel + " || x1 = " + x1 + " || y1 = " + y1 + " || z1 = " + z1);

        vel = veloc(CurVelocity.x + x1 * 0.5f, CurVelocity.y + y1 * 0.5f, 0);
        x2 = dt * Fvx(CurVelocity.x + x1 * 0.5f, vel);
        y2 = dt * Fvy(CurVelocity.y + y1 * 0.5f, vel);
  //      z2 = dt * Fvz(CurVelocity.z + z1 * 0.5f, vel);
        print(CurVelocity);

        vel = veloc(CurVelocity.x + x2 * 0.5f, CurVelocity.y + y2 * 0.5f, 0);
        x3 = dt * Fvx(CurVelocity.x + x2 * 0.5f, vel);
        y3 = dt * Fvy(CurVelocity.y + y2 * 0.5f, vel);
 //       z3 = dt * Fvz(CurVelocity.z + z2 * 0.5f, vel);

        vel = veloc(CurVelocity.x + dt, CurVelocity.y + dt, 0);
        x4 = dt * Fvx(CurVelocity.x + dt, vel);
        y4 = dt * Fvy(CurVelocity.y + dt, vel);
 //       z4 = dt * Fvz(CurVelocity.z + dt, vel);

        CurVelocity.x += (x1 + 2 * x2 + 2 * x3 + x4) / 6;
        CurVelocity.y += (y1 + 2 * y2 + 2 * y3 + y4) / 6;
//      CurVelocity.z += (z1 + 2 * z2 + 2 * z3 + z4) / 6;

    */

        x1 = Fvx(CurVelocity.x, CurVelocity.y);                     
        y1 = Fvy(CurVelocity.x, CurVelocity.y);

        x2 = Fvx(CurVelocity.x + dt * 0.5f * x1, CurVelocity.y + dt * 0.5f);                   
        y2 = Fvy(CurVelocity.x + dt * 0.5f, CurVelocity.y + dt * 0.5f * y1);

        x3 = Fvx(CurVelocity.x + dt * 0.5f * x2, CurVelocity.y + dt * 0.5f);
        y3 = Fvy(CurVelocity.x + dt * 0.5f, CurVelocity.y + dt * 0.5f * y2);

        x4 = Fvx(CurVelocity.x + dt * x3, CurVelocity.y + dt);
        y4 = Fvy(CurVelocity.x + dt, CurVelocity.y +  dt * y3);

        CurVelocity.x += (x1 + 2 * x2 + 2 * x3 + x4) / 6;
        CurVelocity.y += (y1 + 2 * y2 + 2 * y3 + y4) / 6;

        velocityDirection = CurVelocity.normalized; // напрвление, куда "смотрит" пуля 						
        myTrans.forward = velocityDirection;        // поворачиваем снаряд передом по направлению полета 	
        velocityMagnitude = CurVelocity.magnitude;                  // скорость снаряда (длина вектора  CurVelocity) 	

//        EC = mass * velocityMagnitude * velocityMagnitude * 0.5f; 	// кинетическая энергия снаряда  	
    }

    float veloc(float x, float y, float z) {
        float sqrt = new Vector3(x, y, z).magnitude;
  //      print("sqrt = " + sqrt);
        return sqrt;
        
    }

    float Fvx(float x, float y){                        // m/s2
        float sqrt = Mathf.Sqrt(x*x + y*y);
        float fvx = -BulletDeceleration(x * sqrt);
        return fvx;
    }

    float Fvy(float x, float y){                        // m/s2
        float sqrt = Mathf.Sqrt(x * x + y * y);
        float fvy = -9.81f -  BulletDeceleration(y * sqrt);
        return fvy;
    }


    // ***********Clac Velocity Standart********************************************************

    Vector3 acceleration;
	Vector3 velE;
	Vector3 newVelocity;


	Vector3 localWindVelo;
	//	Vector3 MagnusForce;
	void CalcVelocity(){ // рассчет значения, на которое сдвинется пуля, за отрезок времени dt
		acceleration = gravity;  // величина постоянного ускорения (учет гравитации) M/C^2

		velE = CurVelocity + dt * gravity - windVelo;// - Vector3.right * 0.15f;					//	M/C 	направление снаряда без учета сопротивления воздуха


		acceleration -= velE.normalized * BulletDeceleration(velE.magnitude);  				// M/C^2	вычисление сопротивления воздуха
		newVelocity =  dt * acceleration ;													// M/C		величина ускорения с учетом гравитации, направления и силы ветра, сопротивления воздуха 
		CurVelocity += newVelocity;                                                         // M/C		новое смещение пули

        velocityDirection = CurVelocity.normalized; // напрвление, куда "смотрит" пуля 						
        myTrans.forward = velocityDirection;        // поворачиваем снаряд передом по направлению полета 	
        velocityMagnitude = CurVelocity.magnitude;                  // скорость снаряда (длина вектора  CurVelocity) 	

        EC = mass * velocityMagnitude * velocityMagnitude * 0.5f;   // кинетическая энергия снаряда  					
    }

// ***********Bulet MOVE********************************************************		
	void BulletMoveV(){	// двигаем пулю 
		myTrans.position += (CurVelocity * dt); // M += M/C * C
	}

	public float Fv;
	public float BulletDeceleration(float speedd){	// замедление пули, зависящее от баллистического коэффициента,  функции сопротивления и функции плотности воздуха с изменением высоты (не учитывается, т.к. перепад высот мал)
		Fv = (speedd < 340f) ? speedd * speedd * Ro * 0.0001f : (speedd * 0.3333f) - 80f;	// Fv - Функция сопротивления воздуха, зависящая от скорости снаряда.
//		Fv = 0.5f * Ro  * speedd * speedd * A;
		return BallisticFactor * Fv ; // * Hy  , где  Hy = (20000-myTrans.position.y)/(20000+myTrans.position.y) - функция, показывающая изменение плотности воздуха с изменением высоты
	}
//	public float BulletDeceleration(float speedd){
//		float b = 0.5f * 0.15f * A * Ro;
//		return b  * speedd * speedd;
//	}
		

	[ContextMenu("CalcBallisticFactor")]
	public void CalcBallisticFactor(){	// рассчет баллистического коэффициента
		BallisticFactor = (int)( (1000 * calibr * calibr * i / mass) * 1000)/1000f;
	}  


	//*********** CHECK HIT ****************************************
	RaycastHit hit;
	void CheckHits(){
		if (Physics.Raycast(myTrans.position, velocityDirection, out hit, velocityMagnitude * dt )){
			DoDecal (hit.point + (hit.normal * 0.01f));

			if (!DoRicoshet()) { // если не отрикошетили,
				DoPenetration (); // то простреливаем (если простреливается)
			} 
			if (hit.transform.GetComponent<Rigidbody> ()) {
				hit.transform.GetComponent<Rigidbody> ().AddForceAtPosition (velocityDirection * EC * 0.1f, hit.point);
			}

			DebugInfo ();
		}
	}


	Vector3 reflectDirection;
	bool DoRicoshet(){
		float angle = Vector3.Angle (velocityDirection, hit.normal);
		int rand = Random.Range (0, (int)(minRicochetChance * (angle*0.0167f)));
		angle /= 180f;
		if (rand == 0 && velocityMagnitude > 10) { // если отрикошетили
			reflectDirection = Vector3.Reflect (velocityDirection, hit.normal);
			//			Debug.DrawLine (hit.point, hit.point + 1 * hit.normal, Color.red, 10f);
			reflectDirection += myTrans.InverseTransformDirection(new Vector3 (Random.Range (-RicoshetDAngle, RicoshetDAngle), Random.Range (-RicoshetDAngle, RicoshetDAngle), Random.Range (-RicoshetDAngle, RicoshetDAngle)));
			myTrans.position = hit.point;
			myTrans.forward = reflectDirection;
			float newSpeed = velocityMagnitude - (velocityMagnitude * Random.Range (angle - 0.2f, angle + 0.6f));
			if (newSpeed > 4) {
				CurVelocity = reflectDirection.normalized * newSpeed;
				movebul = false;
			} else {
				ReturnToPool ();
			}
			return true;
		}
		return false;
	}

	void DoPenetration(){

		if (hit.collider.GetComponent<Terrain> () == null) {
			float PenetrateCoef = BulletMaterialDensity; // /materialHitDensity
			float PenetrateScale = velocityMagnitude * velocityMagnitude * BallisticFactor * 0.001f * PenetrateCoef * 0.0001f;
			Vector3 Direction = velocityDirection + myTrans.InverseTransformDirection (new Vector3 (Random.Range (-PenetrateDAngle, PenetrateDAngle), Random.Range (-PenetrateDAngle, PenetrateDAngle), 0));
			RaycastHit[] outhits = Physics.RaycastAll (hit.point + Direction.normalized * PenetrateScale, -Direction, PenetrateScale - 0.01f).OrderBy (h => h.distance).ToArray ();
			if (outhits.Length > 0) {
				float depth = (outhits [outhits.Length - 1].point - hit.point).magnitude;
				//				print ("depth = " + depth +  "  || penetrate = " + PenetrateScale );
				if (depth <= PenetrateScale) {
//					myTrans.forward = Direction;
					myTrans.position = hit.point + myTrans.forward * depth * 0.5f; //outhits [outhits.Length - 1].point;
					//					InstDecal (outhits [outhits.Length - 1].point);
					float re = depth / PenetrateScale;
					float newSpeed = velocityMagnitude - (velocityMagnitude * Random.Range (re, re + 0.2f));
					if (newSpeed > 4) {
						CurVelocity = Direction.normalized * newSpeed;
						movebul = false;
						return;
					} 
				} 
			}
		} 
		ReturnToPool ();
	}


	void DoDecal(Vector3 pos){
      

		Quaternion rotat = Quaternion.FromToRotation (Vector3.up, hit.normal);
		PoolManager.instance.ReuseObject(dec,pos,rotat);
	}

	public override void OnObjectReuse(object[] par){
     
		transform.GetComponent<TrailRenderer> ().Clear();
		timer = 0;
		this.speed = (int)par[0];
		this.BallisticFactor =  (float) par[1];
		this.windVelo = (Vector3)par[2];
		this.TypeOfBullet = (TypeOfImmitation)(int)par [3];
		Init ();
		DrawGraph.instance.StartDebug (this);
 
    }

	// ********** DEBUG LOG ****************************************
	bool _debugInfo = true;
	float _distance;
	float _Speed;
	float _EC;
	Vector3 _SdvigOtCentra;

	void DebugInfo(){
		if (_debugInfo) {
			if (hit.transform.tag == "Player"){
				_Speed = velocityMagnitude;
				_EC = Mathf.Round(EC);
				_SdvigOtCentra =  -hit.transform.InverseTransformPoint(hit.point);
                DemonstrationBullet.instance.drawPointInPosition(2 * _SdvigOtCentra);
                _SdvigOtCentra.x = -(int)(_SdvigOtCentra.x * 1000f) / 1000f  * hit.transform.localScale.x;
				_SdvigOtCentra.y = -(int)(_SdvigOtCentra.y * 1000f) / 1000f  * hit.transform.localScale.y;
				_distance = Mathf.Round( Vector3.Distance(_startpos, hit.point) );

				string Info = System.DateTime.Now + "";
				Info += "\n\tНачальная скорость: " + speed;
				float dist = Mathf.Sqrt(_SdvigOtCentra.x * _SdvigOtCentra.x + _SdvigOtCentra.y * _SdvigOtCentra.y)/2f;
				if (dist > 0.445f) {
					Info += "\n\tПопали в: 1";
				} else if (dist > 0.4f) {
					Info += "\n\tПопали в: 2";
				} else if (dist > 0.353f) {
					Info += "\n\tПопали в: 3";
				} else if (dist > 0.305f) {
					Info += "\n\tПопали в: 4";
				} else if (dist > 0.26f) {
					Info += "\n\tПопали в: 5";
				} else if (dist > 0.215f) {
					Info += "\n\tПопали в: 6";
				} else if (dist > 0.175f) {
					Info += "\n\tПопали в: 7";
				} else if (dist > 0.125f) {
					Info += "\n\tПопали в: 8";
				} else if (dist > 0.08f) {
					Info += "\n\tПопали в: 9";
				} else {
					Info += "\n\tПопали в: 10!!!";
				}
				Info += ("\n\tУгол возвышения оружия в момент выстрела: " + _starrot + " градусов\n\tДальность: " + _distance + "\n\tВетер " + windVelo + "м/с\n\tХАРАКТЕРИСТИКИ СНАРЯДА:\n\t-Баллистический коэффициент: " + BallisticFactor + "\n\t-Масса: " + mass+"кг");
				Info += ("\n\tХАРАКТЕРИСТИКИ СНАРЯДА В МОМЕНТ ПОПАДАНИЯ В МИШЕНЬ:\n\t-Скорость: " + _Speed + "м/с\n\t-Кинетическая энергия: " + _EC + "Дж\n\tОтклонение от центра мишени (В метрах): X = " + _SdvigOtCentra.x  + "   Y = " + _SdvigOtCentra.y + "\n\n");
				StreamWriter logFile = new StreamWriter (Application.dataPath + "/log.txt", true);
				logFile.WriteLine (Info);
				logFile.Close ();
			}
		} else {
			return;
		}

	}

/*	void OnDrawGizmosSelected(){
		
		if (!debugTrajectory) {
			return;
		}
		int steps = 200;
		Vector3[] velocyties = new Vector3[steps];
		Vector3 CV = (CurVelocity == Vector3.zero) ? transform.forward * speed : CurVelocity; // M/C
		gravity = Vector3.up * -9.81f; 			// M/C^2
		for (int i = 0; i < steps; i++){

			Vector3 acceleration = gravity; //air resistance, euler forward

			Vector3 velE = CV + Time.fixedDeltaTime * gravity -  windVelo;					//	M/C
			acceleration -= velE.normalized * BulletDecelerationNewAlg(velE.magnitude) ; //air drag		M/C^2
			Vector3 newVelocity = Time.fixedDeltaTime * acceleration  ;						//	M/c

			CV += newVelocity;			// M/C + M/C
			velocyties [i] = CV;
		}
		Vector3 prefpos = transform.position;
		for (int i = 0; i < steps; i++){
			Gizmos.color = Color.red;
			Gizmos.DrawLine (prefpos, prefpos + velocyties [i] * Time.fixedDeltaTime);
			prefpos += velocyties [i]  * Time.fixedDeltaTime;
		}
	} */
/*
    void OnDrawGizmosSelected(){
        Gizmos.color = Color.red;
        Vector3 pointl = this.transform.position;
        float predictedBulletVelocity = speed;
        float stepSize = 0.01f;
   
        float d = -this.transform.localEulerAngles.x;
        d *= Mathf.Deg2Rad;

        for (float step = 0; step < 1; step += stepSize){

            predictedBulletVelocity += (-BallisticFactor * BulletDeceleration(predictedBulletVelocity) - 9.81f * Mathf.Sin(d))*stepSize;
  //          Debug.Log(step + " = " + predictedBulletVelocity);

            d += (-9.81f * Mathf.Cos(d) / predictedBulletVelocity) * stepSize  * Mathf.Rad2Deg;



            Vector3 point2 = pointl + new Vector3(
                0, 
                predictedBulletVelocity * Mathf.Sin(d) * stepSize, 
                predictedBulletVelocity * Mathf.Cos(d) * stepSize
                ); 

            Gizmos.DrawLine(pointl, point2);
            pointl = point2;
        }

    }
    */
}



