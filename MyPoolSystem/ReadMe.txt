* ��� �� ������������, ���� ������ PoolManager �������� �� ������ gameObject � ��������� ���.

    <=> �� ������ ��� ���� ����� �������� ������ PoolObject - ��������� ������� ����� �������� ��� ��������� ������������� � ���������� ������:
	- ��� ������ ����� � ������, ������� ��������� ���������� ������� ��� ���� (�������� ������������ �������) ������� �� ������ ����������, ��������,
	public override void OnObjectReuse(){
		timer = 0;
	}

// ������� ������ - PoolManager.instance.ReuseObject (GameObject prefab, Vector3 position, Quaternion rotation);
// ������� ������ - transform.GetComponent<PoolObject> ().Destroy ();

��������  https://www.youtube.com/watch?v=LhqP3EghQ-Q