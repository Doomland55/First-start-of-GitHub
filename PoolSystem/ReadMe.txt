* что бы использовать, надо скрипт PoolManager повесить на пустой gameObject и заполнить его.

    <=> на объект для пула можно повесить скрипт PoolObject - позволяет сделать ресет значений при повторном использовании и уничтожить объект:
	- для ресета пишем в скрипт, который управляет действиями объекта для пула (например перемещением объекта) функцию со своими значениями, например,
	public override void OnObjectReuse(){
		timer = 0;
	}

// создать объект - PoolManager.instance.ReuseObject (GameObject prefab, Vector3 position, Quaternion rotation);
// удалить объект - transform.GetComponent<PoolObject> ().Destroy ();

источник  https://www.youtube.com/watch?v=LhqP3EghQ-Q
