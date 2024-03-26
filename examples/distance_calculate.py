from haversine import haversine
import numpy as np

# 주어진 좌표값 (위도, 경도)
coordinates = np.array([
    (37.498744, 127.027566),
    (37.496011, 127.037548),
    (37.491210, 127.058528),
    (37.488537, 127.062482),
    (37.486450, 127.031767),
    (37.488624, 127.025840),
    (37.486841, 127.045192),
    (37.493832, 127.056170),
    (37.492000, 127.043000),
    (37.493000, 127.042000),
    (37.505000, 127.090000),
    (37.488000, 127.062000),
    (37.489537, 127.062582)
])

# 맨해튼 거리를 저장할 8x8 행렬 초기화
haversine_manhattan_distances_matrix = np.zeros((13, 13))

# 좌표 간의 맨해튼 거리 계산 (가로 세로 각각에 대해 하버사인 적용)
for i in range(13):
    for j in range(13):
        if i != j:
            # 좌표 간의 가로, 세로 거리 계산
            lat1, lon1 = coordinates[i]
            lat2, lon2 = coordinates[j]
            horizontal_distance = haversine((lat1, lon1), (lat1, lon2))
            vertical_distance = haversine((lat1, lon2), (lat2, lon2))
            
            # 가로 세로 거리의 합을 맨해튼 거리로 사용
            haversine_manhattan_distance = horizontal_distance + vertical_distance
            
            # 맨해튼 거리를 행렬에 저장
            haversine_manhattan_distances_matrix[i, j] = haversine_manhattan_distance

# 결과 출력
print(haversine_manhattan_distances_matrix)


#유클리드
from haversine import haversine
import numpy as np

# 주어진 좌표값 (위도, 경도)
coordinates = np.array([
    (37.498744, 127.027566),
    (37.496011, 127.037548),
    (37.491210, 127.058528),
    (37.488537, 127.062482),
    (37.486450, 127.031767),
    (37.488624, 127.025840),
    (37.486841, 127.045192),
    (37.493832, 127.056170),
    (37.492000, 127.043000),
    (37.493000, 127.042000),
    (37.505000, 127.090000),
    (37.488000, 127.062000),
    (37.489537, 127.062582)
])

# 유클리드 거리를 저장할 8x8 행렬 초기화
haversine_distances_matrix = np.zeros((13, 13))

# 좌표 간의 유클리드 거리 계산 (haversine 함수 적용)
for i in range(13):
    for j in range(13):
        if i != j:
            # 주석으로 표시된 부분이 행렬의 값입니다.
            haversine_distances_matrix[i, j] = haversine(coordinates[i], coordinates[j])

# 결과 출력
# 행렬의 값만 표시
print(haversine_distances_matrix)
