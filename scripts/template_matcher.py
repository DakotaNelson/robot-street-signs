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
    # calculate the difference and its norms
    diff = normalize(img1) - normalize(img2)  # elementwise for scipy arrays
    z_norm = norm(diff.ravel(), 1)  # one norm
    return z_norm


def normalize(arr):
    """
    normalize the image array by taking (each element - mean) / standard dev
    """
    arr_mean = arr.mean()
    std_dev = arr.std()
    return (arr - arr_mean) / std_dev


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
            # load template sign images as grayscale
            self.signs[k] = cv2.imread(filename,0)

            # precompute keypoints and descriptors for the template sign 
            self.kps[k], self.descs[k] = self.sift.detectAndCompute(self.signs[k],None)


    def predict(self, img):
        """
        Uses gather predictions to get visual diffs of the image to each template
        returns a dictionary, keys being signs, values being confidences
        """
        visual_diff = self._gather_predictions(img)

        if visual_diff:
            # inverse - higher confidences for smaller visual differences
            for k in visual_diff:
                visual_diff[k] = 1.0 / visual_diff[k]

            # normalize confidences to sum to one
            total = sum(visual_diff.values())
            for k in visual_diff:
                visual_diff[k] /= total

        # if visual diff was not computed (bad crop, homography could not be computed)
        else:
            # set 0 confidence for all signs
            visual_diff = {k: 0 for k in self.signs.keys()}

        return visual_diff


    def _gather_predictions(self, img):
        """
        Iteratively call _compute_prediction to put together comparisons of one image with each template
        """
        visual_diff = {}
        try:
            kp, des = self.sift.detectAndCompute(img,None)
            for k in self.signs.keys():
                visual_diff[k] = self._compute_prediction(k, img, kp, des)
        except:
            # could not find a homography, because the cropped image is bad.
            return None

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

        
        # visual difference visualization and debugging
        # uncomment the following lines in order to visualize the difference computations

        # f, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(20, 6), sharey=True)
        # norm_im_T = normalize(img_T)
        # norm_sign = normalize(self.signs[k])
        # ax1.imshow(norm_im_T, cmap='gray')
        # ax1.set_title(img_T.dtype)
        # ax2.imshow(norm_sign, cmap='gray')
        # ax2.set_title(self.signs[k].dtype)
        # ax3.imshow(normalize(img_T) - normalize(self.signs[k]), cmap='gray')
        # ax3.imshow(norm_im_T - norm_sign, cmap='gray')
        # ax3.set_title("visual diff: %d" % visual_diff)
        # plt.title("should be" + k)
        # plt.xlabel(visual_diff)
        # plt.ylabel(visual_diff)
        # plt.show()

        return visual_diff


if __name__ == '__main__':
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

    for filename in scenes[:2]:
        scene_img = cv2.imread(filename, 0)
        pred = tm.predict(scene_img)
        print filename
        print pred
