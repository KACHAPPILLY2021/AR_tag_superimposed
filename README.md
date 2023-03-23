<a name="readme-top"></a>

<!-- PROJECT LOGO -->
<br />
<div align="center">


  <h1 align="center">Superimposing Image and 3D cube using Homography </h1>


</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary><h3>Table of Contents</h3></summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#demo">Demo</a></li>
      </ul>
    </li>
    <li>
      <a href="#approach">Approach</a>
      <ul>
        <li><a href="#image-superimposition-pipeline">Image Superimposition Pipeline</a></li>
        <li><a href="#3d-cube-pipeline">3D Cube Pipeline</a></li>
      </ul>
    </li>
    <li>
      <a href="#report">Report</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#usage">Usage</a></li>
      </ul>
    </li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#license">License</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project



This repository contains code to detect a custom April tag which is a fiducial marker. The program detects the April Tag using Fast Fourier Transform, detects its corner using Shi Thomasi corner detection. It utilizes the concepts of homography to superimpose an image over the detected April Tag. A virtual cube is also drawn over the tag using the concepts of projection and calibration matrices. Note that ```no inbuilt OpenCV``` functions were used except for FFT and Shi Thomasi.

The problem statement can be found here. [Problem_Statement](https://github.com/KACHAPPILLY2021/AR_tag_superimposed/blob/main/problem_statement.pdf)


<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Demo

<div align="center">


  <h4 align="center"> Input video (X2 Speed)</h4>


</div>

https://user-images.githubusercontent.com/90359587/224573981-d0489881-6646-47f3-a17c-2d90d94bf7b4.mp4
<div align="center">


  <h4 align="center"> Testudo Superimposed (X2 Speed)</h4>


</div>

https://user-images.githubusercontent.com/90359587/224574177-360f9657-d45e-471f-b04c-9c77705701b9.mp4

[![Youtube](https://img.shields.io/badge/YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white)](https://youtu.be/37zO4EtMOL0)

<div align="center">


  <h4 align="center"> Augmented Reality on April Tag (X2 Speed)</h4>


</div>

https://user-images.githubusercontent.com/90359587/224574290-bb4876b4-7ba6-49e1-96df-f44d76b1119e.mp4

[![Youtube](https://img.shields.io/badge/YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white)](https://youtu.be/pXbqfZ4dJ60)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- Approach -->
## Approach

Brief explanation of the approaches followed for superimposing Testudo image and 3D cube over detected April tag.

### Image Superimposition Pipeline
* The corners of the April Tag were detected using [Shi-Thomasi](https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_shi_tomasi/py_shi_tomasi.html) corner detector.
* The orientation of the April Tag in the video-frame was determined.
* Homography matrix was computed between the detected corners and the Testudo image and ```inverse warping``` was performed to set the value of the pixels in the frame to that of the testudo image. This eliminates the "holes" that may arise if the forward warping was performed.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### 3D Cube Pipeline
The implemented augmented reality application utilizes tracking and pose estimation based on projective transformation.

* Initially, the AprilTag is identified and the position of its corners are obtained across multiple frames. 
*  Subsequently, homography estimation is utilized to calculate the 3D pose of a set of four points in the real world. Instead of simply superimposing an image as done in the first step, a 3D object is rendered in a frame.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- Report -->
## Report

Detailed decription of the pipeline and challenges can be found here. [Report](https://github.com/KACHAPPILLY2021/AR_tag_superimposed/blob/main/ENPM_673_P1_report.pdf)
<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- GETTING STARTED -->
## Getting Started

These are the instructions to get started on the project.
To get a local copy up and running follow these simple steps.

### Prerequisites
* Python 3.6 or above
* Libraries - OpenCV, Numpy
* OS - Linux (tested)


### Usage

1. Clone the repo
   ```sh
   git clone https://github.com/KACHAPPILLY2021/AR_tag_superimposed.git
   ```
2. Open the folder ```AR_tag_superimposed``` in IDE or navigate using terminal
   ```sh
   cd ∼ /AR_tag_superimposed
   ```
3. To terminate any program while output is being generated press ```Q```.
4. To detect edges of the April Tag using FFT:
   ```sh
   python3 1_a.py
   ```
5. To find the orientation of the April Tag by decoding it:
   ```sh
   python3 1_b.py
   ```
6. To superimpose an image over the April Tag:
   ```sh
   python3 2_a.py
   ```
7. To superimpose an 3D over the April Tag:
   ```sh
   python3 2_b.py
   ```
   
<p align="right">(<a href="#readme-top">back to top</a>)</p>




<!-- CONTACT -->
## Contact

Jeffin Johny K - [![MAIL](https://img.shields.io/badge/Gmail-D14836?style=for-the-badge&logo=gmail&logoColor=white)](mailto:jeffinjk@umd.edu)
	
[![portfolio](https://img.shields.io/badge/my_portfolio-000?style=for-the-badge&logo=ko-fi&logoColor=white)](https://github.com/KACHAPPILLY2021)
[![linkedin](https://img.shields.io/badge/linkedin-0A66C2?style=for-the-badge&logo=linkedin&logoColor=white)](http://www.linkedin.com/in/jeffin-johny-kachappilly-0a8597136)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the MIT License. See [MIT](https://choosealicense.com/licenses/mit/) for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/othneildrew/Best-README-Template.svg?style=for-the-badge
[contributors-url]: https://github.com/othneildrew/Best-README-Template/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=for-the-badge
[forks-url]: https://github.com/othneildrew/Best-README-Template/network/members
[stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=for-the-badge
[stars-url]: https://github.com/othneildrew/Best-README-Template/stargazers
[issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=for-the-badge
[issues-url]: https://github.com/othneildrew/Best-README-Template/issues
[license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=for-the-badge
[license-url]: https://github.com/othneildrew/Best-README-Template/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/othneildrew
[product-screenshot]: images/screenshot.png
[Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com
