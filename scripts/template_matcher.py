import numpy as np
import cv2
from matplotlib import pyplot as plt
from scipy.linalg import norm
from scipy import sum, average
import ipdb
import os

def compare_images(img1, img2):
    """
    helper function from:
    http://stackoverflow.com/questions/189943/how-can-i-quantify-difference-between-two-images
    """
    # normalize to compensate for exposure difference, this may be unnecessary
    # consider disabling it
    img1 = normalize(img1)
    img2 = normalize(img2)
    # calculate the difference and its norms
    diff = img1 - img2  # elementwise for scipy arrays
    m_norm = sum(abs(diff))  # Manhattan norm
    z_norm = norm(diff.ravel(), 0)  # Zero norm
    return (m_norm, z_norm)


def normalize(arr):
    """
    helper function from:
    http://stackoverflow.com/questions/189943/how-can-i-quantify-difference-between-two-images
    """
    rng = arr.max()-arr.min()
    amin = arr.min()
    return (arr-amin)*255/rng


class TemplateMatcher(object):

    def __init__ (self, images, min_match_count=10, good_thresh=0.7):
        self.signs = {}
        self.kps = {}
        self.descs = {}
        self.sift = cv2.SIFT()

        # for potential tweaking
        self.min_match_count = min_match_count
        self.good_thresh = good_thresh

        # we have not tweaked, taken directly from tutorial
        self.flann_index_kdtree = 0
        self.trees = 5
        self.checks = 50
        self.ransac_thresh = 5.0
        self.index_params = dict(algorithm = self.flann_index_kdtree, trees = self.trees)
        self.search_params = dict(checks = self.checks)
        self.flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)
        for k, filename in images.iteritems():
            self.signs[k] = cv2.imread(filename,0)
            self.kps[k], self.descs[k] = self.sift.detectAndCompute(self.signs[k],None)

    def predict(self, img, norm=1):
        """
        Uses gather predictions to get visual diffs of the image to each template and then selects and returns the most likely one
        norm: either 0 or 1, where 0 is 'manhattan norm' and 1 is 'L2 norm'
        """
        visual_diff = self._gather_predictions(img)
        likely = None
        lowest = np.inf
        for k in visual_diff.keys():
            if visual_diff[k][norm] < lowest:
                lowest = visual_diff[k][norm]
                likely = k
        return likely

    def _gather_predictions(self, img):
        """
        Iteratively call _compute_prediction to put together comparisons of one image with each template
        """
        visual_diff = {}
        kp, des = self.sift.detectAndCompute(img,None)
        for k in self.signs.keys():
            visual_diff[k] = self._compute_prediction(k, img, kp, des)
        return visual_diff


    def _compute_prediction(self, k, img, kp, des):
        """
        Return comparison values between a template k and given image
        k: template image for comparison
        img: scene image
        kp: keypoints from scene image
        des: descriptors from scene image
        """
        matches = self.flann.knnMatch(des,self.descs[k],k=2)
        good = []
        for m, n in matches:
            if m.distance < self.good_thresh * n.distance:
                good.append(m)
        src_pts = np.float32([kp[match.queryIdx].pt for match in good]).reshape(-1,1,2)
        dst_pts = np.float32([self.kps[k][match.trainIdx].pt for match in good]).reshape(-1,1,2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, self.ransac_thresh)
        img_T = cv2.warpPerspective(img, M, self.signs[k].shape[::-1])
        visual_diff = compare_images(img_T, self.signs[k])
        # plt.imshow(img_T)
        # plt.title(k)
        # plt.xlabel(visual_diff[0])
        # plt.ylabel(visual_diff[1])
        # plt.show()
        return visual_diff

    @staticmethod
    def drawMatches(img1, kp1, img2, kp2, matches, masks):
        """
        http://stackoverflow.com/questions/20259025/module-object-has-no-attribute-drawmatches-opencv-python
        My own implementation of cv2.drawMatches as OpenCV 2.4.9
        does not have this function available but it's supported in
        OpenCV 3.0.0

        This function takes in two images with their associated
        keypoints, as well as a list of DMatch data structure (matches)
        that contains which keypoints matched in which images.

        An image will be produced where a montage is shown with
        the first image followed by the second image beside it.

        Keypoints are delineated with circles, while lines are connected
        between matching keypoints.

        img1,img2 - Grayscale images
        kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint
                  detection algorithms
        matches - A list of matches of corresponding keypoints through any
                  OpenCV keypoint matching algorithm
        masks - a list of which matches are inliers (1?) or outliers (0?)
        """

        # Create a new output image that concatenates the two images together
        # (a.k.a) a montage
        rows1 = img1.shape[0]
        cols1 = img1.shape[1]
        rows2 = img2.shape[0]
        cols2 = img2.shape[1]

        out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

        # Place the first image to the left
        out[:rows1,:cols1] = np.dstack([img1, img1, img1])

        # Place the next image to the right of it
        out[:rows2,cols1:] = np.dstack([img2, img2, img2])

        # For each pair of points we have between both images
        # draw circles, then connect a line between them
        for mask, mat in zip(masks,matches):

            # Get the matching keypoints for each of the images
            img1_idx = mat.queryIdx
            img2_idx = mat.trainIdx

            # x - columns
            # y - rows
            (x1,y1) = kp1[img1_idx].pt
            (x2,y2) = kp2[img2_idx].pt

            # Draw a small circle at both co-ordinates
            # radius 4
            # colour blue
            # thickness = 1

            cv2.circle(out, (int(x1),int(y1)), 4, (255*mask, 0, 255*(1-mask)), 1)
            cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255*mask, 0, 255*(1-mask)), 1)

            # Draw a line in between the two points
            # thickness = 1
            # colour blue
            cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255, 0, 0), 1)


        # Show the image
        cv2.imshow('Matched Features', out)
        cv2.waitKey(0)
        cv2.destroyWindow('Matched Features')

        # Also return the image if you'd like a copy
        return out

if __name__ == '__main__':
    # scene_img = cv2.imread('../images/bin_img_0100.jpg', 0)
    # images = {
    #     "left": '../images/leftturn_box_small.png',
    #     "right": '../images/rightturn_box_small.png',
    #     "uturn": '../images/uturn_box_small.png'
    #     }
    # tm = TemplateMatcher(images)
    # pred = tm.predict(scene_img)
    # print pred
    images = {
        "left": '../images/leftturn_box_small.png',
        "right": '../images/rightturn_box_small.png',
        "uturn": '../images/uturn_box_small.png'
        }
    tm = TemplateMatcher(images)

    scenes = [
        "../images/uturn_scene.jpg",
        "../images/leftturn_scene.jpg",
        "../images/rightturn_scene.jpg"
    ]
    for filename in scenes:
        scene_img = cv2.imread(filename, 0)
        pred = tm.predict(scene_img)
        print pred
        # plt.imshow(tm.img_T)
        # plt.show()
