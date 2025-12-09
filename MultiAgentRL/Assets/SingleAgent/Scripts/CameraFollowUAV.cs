using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using UnityEngine;

public class CameraFollowUAV : MonoBehaviour
{
    // Start is called before the first frame update
    public Transform target; // 要跟随的目标物体

    public float smoothSpeed = 0.125f; // 相机跟随的平滑速度
    public Vector3 offset = new Vector3(0, 3, -5); // 相机与目标物体的偏移量

    private void LateUpdate()
    {
        if (target != null)
        {
            // 计算相机应该到达的目标位置
            Vector3 desiredPosition = target.position + offset;
            // 使用平滑阻尼来实现平滑跟随
            Vector3 smoothedPosition = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed);
            // 设置相机的位置
            transform.position = smoothedPosition;
            // 让相机始终看向目标物体
            transform.LookAt(target);
        }
    }

    // Update is called once per frame
    //void Update()
    //{
    // }
}
