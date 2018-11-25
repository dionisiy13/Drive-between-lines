class DetectCenter:

    @staticmethod
    def get_the_nearest_lines_new(image, center_y, center_x):
        width = center_x * 2
        left = 0
        right = width
        for x in reversed(range(width / 2)):
            if image[center_y, x] == 255.0:
                left = x
            break
        for x in range(width / 2, width):
            if image[center_y, x] == 255.0:
                right = x
                break
        return [left, right, center_y]