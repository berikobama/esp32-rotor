#include <AccelStepper.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "AiEsp32RotaryEncoder.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TMCStepper.h>
#include <Preferences.h>


#define DRIVER_ADDRESS 0b00
#define R_SENSE 0.11f

#define STEPPER_DIR_PIN 12
#define STEPPER_STEP_PIN 14
#define motorInterfaceType 1

#define ROTARY_ENCODER_A_PIN_AZ 26
#define ROTARY_ENCODER_B_PIN_AZ 27
#define ROTARY_ENCODER_BUTTON_PIN_AZ 25
#define ROTARY_ENCODER_VCC_PIN_AZ -1
#define ROTARY_ENCODER_STEPS 4

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

typedef void (*GenericFP)();

struct country {
  String name;
  float lat;
  float lon;
};

country countries[245] = {
  { "Andorra", 42.546245, 1.601554 },
  { "Afghanistan", 33.93911, 67.709953 },
  { "Albania", 41.153332, 20.168331 },
  { "Algeria", 28.033886, 1.659626 },
  { "American Samoa", -14.270972, -170.132217 },
  { "Angola", -11.202692, 17.873887 },
  { "Anguilla", 18.220554, -63.068615 },
  { "Antarctica", -75.250973, -0.071389 },
  { "Antigua and Barbuda", 17.060816, -61.796428 },
  { "Argentina", -38.416097, -63.616672 },
  { "Armenia", 40.069099, 45.038189 },
  { "Aruba", 12.52111, -69.968338 },
  { "Australia", -25.274398, 133.775136 },
  { "Austria", 47.516231, 14.550072 },
  { "Azerbaijan", 40.143105, 47.576927 },
  { "Bahamas", 25.03428, -77.39628 },
  { "Bahrain", 25.930414, 50.637772 },
  { "Bangladesh", 23.684994, 90.356331 },
  { "Barbados", 13.193887, -59.543198 },
  { "Belarus", 53.709807, 27.953389 },
  { "Belgium", 50.503887, 4.469936 },
  { "Belize", 17.189877, -88.49765 },
  { "Benin", 9.30769, 2.315834 },
  { "Bermuda", 32.321384, -64.75737 },
  { "Bhutan", 27.514162, 90.433601 },
  { "Bolivia", -16.290154, -63.588653 },
  { "Bosnia and Herzegovina", 43.915886, 17.679076 },
  { "Botswana", -22.328474, 24.684866 },
  { "Bouvet Island", -54.423199, 3.413194 },
  { "Brazil", -14.235004, -51.92528 },
  { "British Indian Ocean Territory", -6.343194, 71.876519 },
  { "British Virgin Islands", 18.420695, -64.639968 },
  { "Brunei", 4.535277, 114.727669 },
  { "Bulgaria", 42.733883, 25.48583 },
  { "Burkina Faso", 12.238333, -1.561593 },
  { "Burundi", -3.373056, 29.918886 },
  { "Cambodia", 12.565679, 104.990963 },
  { "Cameroon", 7.369722, 12.354722 },
  { "Canada", 56.130366, -106.346771 },
  { "Cape Verde", 16.002082, -24.013197 },
  { "Cayman Islands", 19.513469, -80.566956 },
  { "Central African Republic", 6.611111, 20.939444 },
  { "Chad", 15.454166, 18.732207 },
  { "Chile", -35.675147, -71.542969 },
  { "China", 35.86166, 104.195397 },
  { "Christmas Island", -10.447525, 105.690449 },
  { "Cocos Islands", -12.164165, 96.870956 },
  { "Colombia", 4.570868, -74.297333 },
  { "Comoros", -11.875001, 43.872219 },
  { "Congo", -4.038333, 21.758664 },
  { "Congo", -0.228021, 15.827659 },
  { "Cook Islands", -21.236736, -159.777671 },
  { "Costa Rica", 9.748917, -83.753428 },
  { "Cote dIvoire", 7.539989, -5.54708 },
  { "Croatia", 45.1, 15.2 },
  { "Cuba", 21.521757, -77.781167 },
  { "Cyprus", 35.126413, 33.429859 },
  { "Czech Republic", 49.817492, 15.472962 },
  { "Denmark", 56.26392, 9.501785 },
  { "Djibouti", 11.825138, 42.590275 },
  { "Dominica", 15.414999, -61.370976 },
  { "Dominican Republic", 18.735693, -70.162651 },
  { "Ecuador", -1.831239, -78.183406 },
  { "Egypt", 26.820553, 30.802498 },
  { "El Salvador", 13.794185, -88.89653 },
  { "Equatorial Guinea", 1.650801, 10.267895 },
  { "Eritrea", 15.179384, 39.782334 },
  { "Estonia", 58.595272, 25.013607 },
  { "Ethiopia", 9.145, 40.489673 },
  { "Falkland Islands", -51.796253, -59.523613 },
  { "Faroe Islands", 61.892635, -6.911806 },
  { "Fiji", -16.578193, 179.414413 },
  { "Finland", 61.92411, 25.748151 },
  { "France", 46.227638, 2.213749 },
  { "French Guiana", 3.933889, -53.125782 },
  { "French Polynesia", -17.679742, -149.406843 },
  { "French Southern Territories", -49.280366, 69.348557 },
  { "Gabon", -0.803689, 11.609444 },
  { "Gambia", 13.443182, -15.310139 },
  { "Gaza Strip", 31.354676, 34.308825 },
  { "Georgia", 42.315407, 43.356892 },
  { "Germany", 51.165691, 10.451526 },
  { "Ghana", 7.946527, -1.023194 },
  { "Gibraltar", 36.137741, -5.345374 },
  { "Greece", 39.074208, 21.824312 },
  { "Greenland", 71.706936, -42.604303 },
  { "Grenada", 12.262776, -61.604171 },
  { "Guadeloupe", 16.995971, -62.067641 },
  { "Guam", 13.444304, 144.793731 },
  { "Guatemala", 15.783471, -90.230759 },
  { "Guernsey", 49.465691, -2.585278 },
  { "Guinea", 9.945587, -9.696645 },
  { "Guinea-Bissau", 11.803749, -15.180413 },
  { "Guyana", 4.860416, -58.93018 },
  { "Haiti", 18.971187, -72.285215 },
  { "Heard Island and McDonald Islands", -53.08181, 73.504158 },
  { "Honduras", 15.199999, -86.241905 },
  { "Hong Kong", 22.396428, 114.109497 },
  { "Hungary", 47.162494, 19.503304 },
  { "Iceland", 64.963051, -19.020835 },
  { "India", 20.593684, 78.96288 },
  { "Indonesia", -0.789275, 113.921327 },
  { "Iran", 32.427908, 53.688046 },
  { "Iraq", 33.223191, 43.679291 },
  { "Ireland", 53.41291, -8.24389 },
  { "Isle of Man", 54.236107, -4.548056 },
  { "Israel", 31.046051, 34.851612 },
  { "Italy", 41.87194, 12.56738 },
  { "Jamaica", 18.109581, -77.297508 },
  { "Japan", 36.204824, 138.252924 },
  { "Jersey", 49.214439, -2.13125 },
  { "Jordan", 30.585164, 36.238414 },
  { "Kazakhstan", 48.019573, 66.923684 },
  { "Kenya", -0.023559, 37.906193 },
  { "Kiribati", -3.370417, -168.734039 },
  { "Kosovo", 42.602636, 20.902977 },
  { "Kuwait", 29.31166, 47.481766 },
  { "Kyrgyzstan", 41.20438, 74.766098 },
  { "Laos", 19.85627, 102.495496 },
  { "Latvia", 56.879635, 24.603189 },
  { "Lebanon", 33.854721, 35.862285 },
  { "Lesotho", -29.609988, 28.233608 },
  { "Liberia", 6.428055, -9.429499 },
  { "Libya", 26.3351, 17.228331 },
  { "Liechtenstein", 47.166, 9.555373 },
  { "Lithuania", 55.169438, 23.881275 },
  { "Luxembourg", 49.815273, 6.129583 },
  { "Macau", 22.198745, 113.543873 },
  { "Macedonia", 41.608635, 21.745275 },
  { "Madagascar", -18.766947, 46.869107 },
  { "Malawi", -13.254308, 34.301525 },
  { "Malaysia", 4.210484, 101.975766 },
  { "Maldives", 3.202778, 73.22068 },
  { "Mali", 17.570692, -3.996166 },
  { "Malta", 35.937496, 14.375416 },
  { "Marshall Islands", 7.131474, 171.184478 },
  { "Martinique", 14.641528, -61.024174 },
  { "Mauritania", 21.00789, -10.940835 },
  { "Mauritius", -20.348404, 57.552152 },
  { "Mayotte", -12.8275, 45.166244 },
  { "Mexico", 23.634501, -102.552784 },
  { "Micronesia", 7.425554, 150.550812 },
  { "Moldova", 47.411631, 28.369885 },
  { "Monaco", 43.750298, 7.412841 },
  { "Mongolia", 46.862496, 103.846656 },
  { "Montenegro", 42.708678, 19.37439 },
  { "Montserrat", 16.742498, -62.187366 },
  { "Morocco", 31.791702, -7.09262 },
  { "Mozambique", -18.665695, 35.529562 },
  { "Myanmar", 21.913965, 95.956223 },
  { "Namibia", -22.95764, 18.49041 },
  { "Nauru", -0.522778, 166.931503 },
  { "Nepal", 28.394857, 84.124008 },
  { "Netherlands", 52.132633, 5.291266 },
  { "Netherlands Antilles", 12.226079, -69.060087 },
  { "New Caledonia", -20.904305, 165.618042 },
  { "New Zealand", -40.900557, 174.885971 },
  { "Nicaragua", 12.865416, -85.207229 },
  { "Niger", 17.607789, 8.081666 },
  { "Nigeria", 9.081999, 8.675277 },
  { "Niue", -19.054445, -169.867233 },
  { "Norfolk Island", -29.040835, 167.954712 },
  { "North Korea", 40.339852, 127.510093 },
  { "Northern Mariana Islands", 17.33083, 145.38469 },
  { "Norway", 60.472024, 8.468946 },
  { "Oman", 21.512583, 55.923255 },
  { "Pakistan", 30.375321, 69.345116 },
  { "Palau", 7.51498, 134.58252 },
  { "Palestinian Territories", 31.952162, 35.233154 },
  { "Panama", 8.537981, -80.782127 },
  { "Papua New Guinea", -6.314993, 143.95555 },
  { "Paraguay", -23.442503, -58.443832 },
  { "Peru", -9.189967, -75.015152 },
  { "Philippines", 12.879721, 121.774017 },
  { "Pitcairn Islands", -24.703615, -127.439308 },
  { "Poland", 51.919438, 19.145136 },
  { "Portugal", 39.399872, -8.224454 },
  { "Puerto Rico", 18.220833, -66.590149 },
  { "Qatar", 25.354826, 51.183884 },
  { "Reunion", -21.115141, 55.536384 },
  { "Romania", 45.943161, 24.96676 },
  { "Russia", 61.52401, 105.318756 },
  { "Rwanda", -1.940278, 29.873888 },
  { "Saint Helena", -24.143474, -10.030696 },
  { "Saint Kitts and Nevis", 17.357822, -62.782998 },
  { "Saint Lucia", 13.909444, -60.978893 },
  { "Saint Pierre and Miquelon", 46.941936, -56.27111 },
  { "Saint Vincent and the Grenadines", 12.984305, -61.287228 },
  { "Samoa", -13.759029, -172.104629 },
  { "San Marino", 43.94236, 12.457777 },
  { "Sao Tome and Principe", 0.18636, 6.613081 },
  { "Saudi Arabia", 23.885942, 45.079162 },
  { "Senegal", 14.497401, -14.452362 },
  { "Serbia", 44.016521, 21.005859 },
  { "Seychelles", -4.679574, 55.491977 },
  { "Sierra Leone", 8.460555, -11.779889 },
  { "Singapore", 1.352083, 103.819836 },
  { "Slovakia", 48.669026, 19.699024 },
  { "Slovenia", 46.151241, 14.995463 },
  { "Solomon Islands", -9.64571, 160.156194 },
  { "Somalia", 5.152149, 46.199616 },
  { "South Africa", -30.559482, 22.937506 },
  { "South Georgia and the South Sandwich Islands", -54.429579, -36.587909 },
  { "South Korea", 35.907757, 127.766922 },
  { "Spain", 40.463667, -3.74922 },
  { "Sri Lanka", 7.873054, 80.771797 },
  { "Sudan", 12.862807, 30.217636 },
  { "Suriname", 3.919305, -56.027783 },
  { "Svalbard and Jan Mayen", 77.553604, 23.670272 },
  { "Swaziland", -26.522503, 31.465866 },
  { "Sweden", 60.128161, 18.643501 },
  { "Switzerland", 46.818188, 8.227512 },
  { "Syria", 34.802075, 38.996815 },
  { "Taiwan", 23.69781, 120.960515 },
  { "Tajikistan", 38.861034, 71.276093 },
  { "Tanzania", -6.369028, 34.888822 },
  { "Thailand", 15.870032, 100.992541 },
  { "Timor-Leste", -8.874217, 125.727539 },
  { "Togo", 8.619543, 0.824782 },
  { "Tokelau", -8.967363, -171.855881 },
  { "Tonga", -21.178986, -175.198242 },
  { "Trinidad and Tobago", 10.691803, -61.222503 },
  { "Tunisia", 33.886917, 9.537499 },
  { "Turkey", 38.963745, 35.243322 },
  { "Turkmenistan", 38.969719, 59.556278 },
  { "Turks and Caicos Islands", 21.694025, -71.797928 },
  { "Tuvalu", -7.109535, 177.64933 },
  { "U.S. Virgin Islands", 18.335765, -64.896335 },
  { "Uganda", 1.373333, 32.290275 },
  { "Ukraine", 48.379433, 31.16558 },
  { "United Arab Emirates", 23.424076, 53.847818 },
  { "United Kingdom", 55.378051, -3.435973 },
  { "United States", 37.09024, -95.712891 },
  { "Uruguay", -32.522779, -55.765835 },
  { "Uzbekistan", 41.377491, 64.585262 },
  { "Vanuatu", -15.376706, 166.959158 },
  { "Vatican City", 41.902916, 12.453389 },
  { "Venezuela", 6.42375, -66.58973 },
  { "Vietnam", 14.058324, 108.277199 },
  { "Wallis and Futuna", -13.768752, -177.156097 },
  { "Western Sahara", 24.215527, -12.885834 },
  { "Yemen", 15.552727, 48.516388 },
  { "Zambia", -13.133897, 27.849332 },
  { "Zimbabwe", -19.015438, 29.154857 }
};

struct prefix {
  String name;
  float lat;
  float lon;
};

prefix prefixes[320] = {
  { "1A", 41.902782, 12.496365 },
  { "1S", 10.37911, 114.46403 },
  { "2I", 53.41291, -8.24389 },
  { "3A", 43.750298, 7.412841 },
  { "3B6, 3B7", 21.14514, 55.28018 },
  { "3B8", -20.348404, 57.552152 },
  { "3B9", 28.38067, -16.51437 },
  { "3C0", -1.43507, 5.63592 },
  { "3C1", 1.650801, 10.267895 },
  { "3D2/C", -21.7499999999999, 174.58333 },
  { "3D2/F", -16.578193, 179.414413 },
  { "3D2/R", -12.5048799999999, 177.08917 },
  { "3DA", -26.522503, 31.465866 },
  { "3V", 33.886917, 9.537499 },
  { "3W", 14.058324, 108.277199 },
  { "3X", 14.058324, 108.277199 },
  { "3Y", -54.4182601429999, 3.36434315400003 },
  { "4J-4K", 40.143105, 47.576927 },
  { "4L", 42.315407, 43.356892 },
  { "4O", 42.708678, 19.37439 },
  { "4P-4S", 7.873054, 80.771797 },
  { "4W", -8.82031921699996, 125.85240272 },
  { "4X,4Z", 31.046051, 34.851612 },
  { "5A", 26.3351, 17.228331 },
  { "5B", 35.126413, 33.429859 },
  { "5C-5D", 31.791702, -7.09262 },
  { "5H-5I", -6.369028, 34.888822 },
  { "5J-5K", 4.570868, -74.297333 },
  { "5N-5O", 9.081999, 8.675277 },
  { "5R-5S", -18.766947, 46.869107 },
  { "5T", 21.00789, -10.940835 },
  { "5U", 17.607789, 8.081666 },
  { "5V", 8.619543, 0.824782 },
  { "5W", -13.759029, -172.104629 },
  { "5X", 1.373333, 32.290275 },
  { "5Y-5Z", -0.023559, 37.906193 },
  { "6D-6J", 23.634501, -102.552784 },
  { "6K-6N", 35.907757, 127.766922 },
  { "6V-6W", 14.497401, -14.452362 },
  { "6Y", 18.109581, -77.297508 },
  { "7J-7N", 36.204824, 138.252924 },
  { "7O", 15.552727, 48.516388 },
  { "7P", -29.609988, 28.233608 },
  { "7Q", -13.254308, 34.301525 },
  { "7S,8S", 60.128161, 18.643501 },
  { "7T-7Y", 28.033886, 1.659626 },
  { "7Z", 23.885942, 45.079162 },
  { "8J-8N", 36.204824, 138.252924 },
  { "8P", 13.193887, -59.543198 },
  { "8Q", 3.202778, 73.22068 },
  { "8R", 4.860416, -58.93018 },
  { "9A", 45.1, 15.2 },
  { "9G", 7.946527, -1.023194 },
  { "9H", 35.937496, 14.375416 },
  { "9I-9J", -13.133897, 27.849332 },
  { "9K", 29.31166, 47.481766 },
  { "9L", 8.460555, -11.779889 },
  { "9M2-9M4", 4.210484, 101.975766 },
  { "9M6-9M8", 4.210484, 101.975766 },
  { "9M0", 7.67362263200004, 111.663177266 },
  { "9N", 28.394857, 84.124008 },
  { "9Q-9T", -0.228021, 15.827659 },
  { "9U", -3.373056, 29.918886 },
  { "9V", 1.352083, 103.819836 },
  { "9X", -1.940278, 29.873888 },
  { "9Y-9Z", 10.691803, -61.222503 },
  { "A2", -22.328474, 24.684866 },
  { "A3", -21.178986, -175.198242 },
  { "A4", 21.512583, 55.923255 },
  { "A5", 27.514162, 90.433601 },
  { "A6", 23.424076, 53.847818 },
  { "A7", 25.354826, 51.183884 },
  { "A9", 25.930414, 50.637772 },
  { "AA-AK", 37.09024, -95.712891 },
  { "AP", 30.375321, 69.345116 },
  { "B", 35.86166, 104.195397 },
  { "BV, BN-BX ", 23.69781, 120.960515 },
  { "BY", 35.86166, 104.195397 },
  { "C2", -0.522778, 166.931503 },
  { "C3", 42.546245, 1.601554 },
  { "C4", 35.126413, 33.429859 },
  { "C5", 13.443182, -15.310139 },
  { "C6", 25.03428, -77.39628 },
  { "C8-C9", -18.665695, 35.529562 },
  { "CA-CE", -35.675147, -71.542969 },
  { "CE9", -75.250973, -0.071389 },
  { "CF-CK", 56.130366, -106.346771 },
  { "CL-CM", 21.521757, -77.781167 },
  { "CN", 31.791702, -7.09262 },
  { "CO", 21.521757, -77.781167 },
  { "CP", -16.290154, -63.588653 },
  { "CT", 39.399872, -8.224454 },
  { "CT3", 32.65871, -16.9218299999999 },
  { "CU", 37.74058, -25.6728199999999 },
  { "CV-CX", -32.522779, -55.765835 },
  { "CY9", 57.12342, -170.274919999999 },
  { "D2-D3", -11.202692, 17.873887 },
  { "D4", 16.002082, -24.013197 },
  { "D6", -11.875001, 43.872219 },
  { "D7-D9", 35.907757, 127.766922 },
  { "DA-DR", 51.165691, 10.451526 },
  { "DS-DT", 35.907757, 127.766922 },
  { "DU-DZ", 12.879721, 121.774017 },
  { "E2", 15.870032, 100.992541 },
  { "E3", 15.179384, 39.782334 },
  { "E4", 31.912721033, 35.204684814 },
  { "E5", -21.2346579969999, -159.778020875999 },
  { "E6", -19.054445, -169.867233 },
  { "E7", 44.168254123, 17.785249916 },
  { "EA-EH", 40.463667, -3.74922 },
  { "EA6-EH6", 39.574227949, 2.91264849900005 },
  { "EA8-EH8", 28, -15.4999999999999 },
  { "EA9-EH9", 40.463667, -3.74922 },
  { "EI-EJ", 53.41291, -8.24389 },
  { "EK", 40.069099, 45.038189 },
  { "EL", 6.428055, -9.429499 },
  { "EM-EO", 48.379433, 31.16558 },
  { "EP", 32.427908, 53.688046 },
  { "ER", 47.411631, 28.369885 },
  { "ES", 58.595272, 25.013607 },
  { "ET", 9.145, 40.489673 },
  { "EU-EW", 53.709807, 27.953389 },
  { "EX", 41.43197845, 74.467594244 },
  { "EY", 39, 71 },
  { "EZ", 38.969719, 59.556278 },
  { "F", 46.227638, 2.213749 },
  { "FG", 16.995971, -62.067641 },
  { "FH", -12.8275, 45.166244 },
  { "FJ ", 17.902808611, -62.8297969189999 },
  { "FK", -20.904305, 165.618042 },
  { "FK/C ", -16.3499999999999, 43.93333 },
  { "FM", 14.641528, -61.024174 },
  { "FO", -17.679742, -149.406843 },
  { "FO/A", -33.9331999999999, 150.81202 },
  { "FO/C", 10.29216, -109.207219999999 },
  { "FO/M", 49.97445, 9.14529000000004 },
  { "FP", 46.856133163, -56.3228299999999 },
  { "FR", -21.1331873459999, 55.532444451 },
  { "FS", 18.07951737, -63.0610812399999 },
  { "FY", 3.933889, -53.125782 },
  { "G", 55.378051, -3.435973 },
  { "GD", 54.236107, -4.548056 },
  { "GI, GN ", 53.41291, -8.24389 },
  { "GJ, GH ", 49.214439, -2.13125 },
  { "GM ", 56.840024906, -4.18129654199992 },
  { "GP, GU", 49.465691, -2.585278 },
  { "GS ", 56.840024906, -4.18129654199992 },
  { "GT ", 54.236107, -4.548056 },
  { "GW", 52.337163856, -3.76522262499997 },
  { "GX", 55.378051, -3.435973 },
  { "H6-H7", 12.865416, -85.207229 },
  { "HA", 47.162494, 19.503304 },
  { "HB", 46.818188, 8.227512 },
  { "HB0", 47.166, 9.555373 },
  { "HC-HD", -1.831239, -78.183406 },
  { "HC8-HD8", 5.68, -90.4999999999999 },
  { "HF0 ", -62.0366735599367, -58.3201039291529 },
  { "HG", 47.162494, 19.503304 },
  { "HH", 18.971187, -72.285215 },
  { "HI", 18.735693, -70.162651 },
  { "HL", 35.907757, 127.766922 },
  { "HO-HP", 8.537981, -80.782127 },
  { "HQ-HR", 15.199999, -86.241905 },
  { "HS", 15.870032, 100.992541 },
  { "HV", 41.90225, 12.4533 },
  { "HZ", 23.885942, 45.079162 },
  { "I", 41.87194, 12.56738 },
  { "IM0, IS0", 40.087776372, 9.03049565300006 },
  { "J2", 11.825138, 42.590275 },
  { "J3", 12.262776, -61.604171 },
  { "J4", 39.074208, 21.824312 },
  { "J7", 15.414999, -61.370976 },
  { "J8", 13.201840619, -61.2007514719999 },
  { "JA-JS", 36.204824, 138.252924 },
  { "JD1", 24.28844, 153.98133 },
  { "JD1", 27.09426397, 142.19184768 },
  { "JT-JV", 46.862496, 103.846656 },
  { "JW", 78.858451403, 18.445815169 },
  { "JX", 71.015237523, -8.38499278299997 },
  { "JY", 30.585164, 36.238414 },
  { "K", 37.09024, -95.712891 },
  { "KC4", -75.250973, -0.071389 },
  { "KG4", 47.79576, 22.8853 },
  { "KH1", 0.807446010000035, -176.617484549999 },
  { "KH2", 13.444304, 144.793731 },
  { "KH3", 43.62479, -72.3295399999999 },
  { "KH6-KH7", 21.439542579, -157.943631929999 },
  { "KH7K", 28.39344, -178.2924 },
  { "KH8", -14.270972, -170.132217 },
  { "KH9", 19.29911, 166.62393 },
  { "KL", 64.808087898, -151.004157814999 },
  { "KP1", 18.40281, -75.0132099999999 },
  { "KP2", 17.728114454, -64.8190874999999 },
  { "KP3-KP4", 18.220833, -66.590149 },
  { "LA-LN", 60.472024, 8.468946 },
  { "LO-LW, LU", -38.416097, -63.616672 },
  { "LU/Z", -62.04, -58.32010393 },
  { "LX", 49.815273, 6.129583 },
  { "LY", 55.169438, 23.881275 },
  { "LZ", 42.733883, 25.48583 },
  { "M", 55.378051, -3.435973 },
  { "MD,MI, MN", 54.236107, -4.548056 },
  { "MJ", 49.214439, -2.13125 },
  { "MM", 56.840024906, -4.18129654199992 },
  { "MU", 49.465691, -2.585278 },
  { "MW", 52.337163856, -3.76522262499997 },
  { "N", 55.378051, -3.435973 },
  { "NH6-NH7", 21.439542579, -157.943631929999 },
  { "NL", 64.808087898, -151.004157814999 },
  { "OA-OC", -9.189967, -75.015152 },
  { "OD", 33.854721, 35.862285 },
  { "OE", 47.516231, 14.550072 },
  { "OF-OI", 61.92411, 25.748151 },
  { "OH0", 60.25, 20 },
  { "OJ0", 26.57125, 50.02862 },
  { "OK-OL", 49.817492, 15.472962 },
  { "OM", 49.817492, 15.472962 },
  { "ON-OT", 50.503887, 4.469936 },
  { "OX", 71.706936, -42.604303 },
  { "OY ", 62.071116757, -6.89230317399994 },
  { "OZ", 56.26392, 9.501785 },
  { "P2 ", -6.314993, 143.95555 },
  { "P3", 35.126413, 33.429859 },
  { "P4", 12.52111, -69.968338 },
  { "P5", 40.339852, 127.510093 },
  { "PA-PI", 52.132633, 5.291266 },
  { "PP-PY", -14.235004, -51.92528 },
  { "PZ", 3.919305, -56.027783 },
  { "R1-7", 55, 40 },
  { "R1AN", -75.250973, -0.071389 },
  { "R8-0", 55.74522, 37.58181 },
  { "S0", 24.215527, -12.885834 },
  { "S2", 23.684994, 90.356331 },
  { "S5", 46.151241, 14.995463 },
  { "S7", -4.679574, 55.491977 },
  { "SA-SM", 60.128161, 18.643501 },
  { "SN-SR", 51.919438, 19.145136 },
  { "ST", 12.862807, 30.217636 },
  { "SU", 26.820553, 30.802498 },
  { "SV-SZ", 39.074208, 21.824312 },
  { "SV/A", 37.40514, -79.0500199999999 },
  { "SV5", 36.84331, 27.14809 },
  { "SV9", 35.229066525, 24.845170005 },
  { "T19", -11.9999999999999, 96.83333 },
  { "T2 ", -7.109535, 177.64933 },
  { "T4", 21.521757, -77.781167 },
  { "T5", 5.152149, 46.199616 },
  { "T6", 33.93911, 67.709953 },
  { "T7", 43.94236, 12.457777 },
  { "T8", 7.51498, 134.58252 },
  { "TA-TC", 38.963745, 35.243322 },
  { "TD", 15.783471, -90.230759 },
  { "TE", 9.748917, -83.753428 },
  { "TF", 64.963051, -19.020835 },
  { "TG", 15.783471, -90.230759 },
  { "TI ", 9.748917, -83.753428 },
  { "TJ", 7.369722, 12.354722 },
  { "TK", 42.15171218, 9.10613235000005 },
  { "TN", -0.228021, 15.827659 },
  { "TR", -0.803689, 11.609444 },
  { "TT", 15.454166, 18.732207 },
  { "TU", 7.62918254100003, -5.55472858799993 },
  { "TY", 9.30769, 2.315834 },
  { "TZ", 17.570692, -3.996166 },
  { "UA-UI 1-7", 55, 40 },
  { "UA-UI 8-0", 55.74522, 37.58181 },
  { "UA2F, UA2K", 54.70907, 20.50928 },
  { "UJ-UM", 41.377491, 64.585262 },
  { "UN-UQ", 48.183106164, 67.195045482 },
  { "UR-UZ", 48.379433, 31.16558 },
  { "V2", 17.077664637, -61.7987101209999 },
  { "V3", 17.189877, -88.49765 },
  { "V4", 17.339711689, -62.7656038459999 },
  { "V5 ", -22.95764, 18.49041 },
  { "V6", 7.425554, 150.550812 },
  { "V7", 7.06911171200005, 171.29531 },
  { "V8", 4.535277, 114.727669 },
  { "VA-VG", 56.130366, -106.346771 },
  { "VK", -25.274398, 133.775136 },
  { "VO", 56.130366, -106.346771 },
  { "VP9", 32.321384, -64.75737 },
  { "VQ9", -7.29926169299994, 72.3870325 },
  { "VR", 22.396428, 114.109497 },
  { "VU", 20.593684, 78.96288 },
  { "VU4", 12.472918209, 92.780975 },
  { "VU7", 11.144118875, 72.048025 },
  { "VY", 56.130366, -106.346771 },
  { "W", 37.09024, -95.712891 },
  { "WH6-WH7", 21.439542579, -157.943631929999 },
  { "WL", 64.808087898, -151.004157814999 },
  { "XA-XI", 23.634501, -102.552784 },
  { "XA-XI", 55.63417, -131.297499999999 },
  { "XT", 12.238333, -1.561593 },
  { "XU", 12.565679, 104.990963 },
  { "XV", 14.058324, 108.277199 },
  { "XW", 19.85627, 102.495496 },
  { "XX9", 22.157818454, 113.560352359 },
  { "XY-XZ", 21.100659454, 96.503737821 },
  { "YA", 33.93911, 67.709953 },
  { "YB-YH", -0.789275, 113.921327 },
  { "YI", 33.223191, 43.679291 },
  { "YJ", -15.376706, 166.959158 },
  { "YK", 34.802075, 38.996815 },
  { "YL", 56.879635, 24.603189 },
  { "YM", 38.963745, 35.243322 },
  { "YN", 12.865416, -85.207229 },
  { "YO-YR", 45.943161, 24.96676 },
  { "YS", 13.794185, -88.89653 },
  { "YT-YU", 44.016521, 21.005859 },
  { "YV-YY", 6.42375, -66.58973 },
  { "YV0", 41.36695, -8.40499999999997 },
  { "Z2", -19.015438, 29.154857 },
  { "Z3", 41.608635, 21.745275 },
  { "Z6", 42.602636, 20.902977 },
  { "Z8", 7.26346091400006, 30.343765666 },
  { "ZA", 41.153332, 20.168331 },
  { "ZB", 36.137741, -5.345374 },
  { "ZP", -23.442503, -58.443832 },
  { "ZR-ZU", -30.559482, 22.937506 },
  { "ZS8", 51.51347, -0.130559999999945 },
};

void stop();
void direct_control();
void set_zero();
void handle_country();
void handle_prefix();
void country_menu();
void prefix_menu_fun();
void set_lat();
void set_lon();

struct menu_item {
  String entry;
  GenericFP func;
};

struct menu {
  String header;
  int position;
  menu_item* menu_items;
  int num_items;
};

menu_item main_menu_items[7] = {
  { "Stop", &stop },
  { "Azimuth", &direct_control },
  { "Country", &country_menu },
  { "Prefix", &prefix_menu_fun },
  { "Set 0", &set_zero },
  { "Set lat", &set_lat },
  { "Set lon", &set_lon },
};

menu main_menu = { "Main:", 0, main_menu_items, 7 };
menu current_menu = main_menu;

menu direct_menu = { "Azimuth", NULL, 0 };

menu_item by_country_items[245];
menu countries_menu = { "Country", 0, by_country_items, 245 };

menu_item by_prefix_items[320];
menu prefix_menu = { "Prefix", 0, by_prefix_items, 320 };


AccelStepper stepper = AccelStepper(motorInterfaceType, STEPPER_STEP_PIN, STEPPER_DIR_PIN);
TMC2209Stepper TMCdriver(&Serial2, R_SENSE, DRIVER_ADDRESS);  // Create TMC driver

AiEsp32RotaryEncoder encoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN_AZ, ROTARY_ENCODER_B_PIN_AZ, ROTARY_ENCODER_BUTTON_PIN_AZ, ROTARY_ENCODER_VCC_PIN_AZ, ROTARY_ENCODER_STEPS);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const float steps_per_degree = ((1600.0) / 360.0) * 50;
String serial_in, az_in, el_in;
float az = 0;
float el = 0;
float az_old = 0;
unsigned int split_index = 0;
bool encoder_changed = true;

Preferences preferences;

float pos_lat = 51.34;
float pos_lon = 12.36;

const int centreX = 107;
const int centreY = 43;
const int radius = 20;

TaskHandle_t Core0TaskHnd;
TaskHandle_t Core1TaskHnd;


void rotary_onButtonClick() {
  static unsigned long lastTimePressed = 0;  // Soft debouncing
  if (millis() - lastTimePressed < 600) {
    return;
  }
  lastTimePressed = millis();
  if (current_menu.header == "Azimuth") {
    current_menu = main_menu;
    encoder.setBoundaries(0, current_menu.num_items - 1, false);
    encoder.setEncoderValue(current_menu.position);
  } else if (current_menu.header == "Country" || current_menu.header == "Prefix") {
    current_menu.menu_items[current_menu.position].func();
    direct_control();
  } else {
    current_menu.position = encoder.readEncoder();
    current_menu.menu_items[current_menu.position].func();
  }
}

void rotary_loop() {
  encoder_changed = false;
  if (encoder.encoderChanged()) {
    if (current_menu.header == "Azimuth") {
      from_to((float)encoder.readEncoder());
      direct_control();
    } else {
      current_menu.position = encoder.readEncoder();
    }
  }
}

void IRAM_ATTR readEncoderISR() {
  encoder.readEncoder_ISR();
  encoder_changed = true;
}

void init_stepper_uart() {
  Serial2.begin(115200, SERIAL_8N1, 3, 1);
  delay(500);
  TMCdriver.begin();
  TMCdriver.toff(5);
  TMCdriver.rms_current(650);
  TMCdriver.microsteps(1);

  TMCdriver.pwm_freq(8);

  TMCdriver.en_spreadCycle(false);
  TMCdriver.pwm_autoscale(true);
  delay(500);
  Serial2.end();
}

void init_stepper() {
  stepper.setMaxSpeed(10000);
  stepper.setAcceleration(1000);
  delay(1000);
}

void init_encoder() {
  encoder.begin();
  encoder.setup(readEncoderISR);
  encoder.setBoundaries(0, current_menu.num_items - 1, false);
}

void init_display() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  delay(500);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 8);
  // Display static text
  display.println("Booting...\n");
  display.setTextSize(4);
  display.println("DF9ES");
  display.display();
  delay(800);
}


void setup() {
  preferences.begin("settings", false);
  
  pos_lat = preferences.getFloat("pos_lat");
  pos_lon = preferences.getFloat("pos_lon");

  for (int i = 0; i < 245; i++) {
    by_country_items[i] = { countries[i].name, &handle_country };
  }

  for (int i = 0; i < 245; i++) {
    by_prefix_items[i] = { prefixes[i].name, &handle_prefix };
  }

  init_display();
  init_encoder();

  serial_in.reserve(50);
  Serial.begin(115200);

  init_stepper();

  xTaskCreatePinnedToCore(CoreTask0, "CPU_0", 4096, NULL, 1, &Core0TaskHnd, 0);
  xTaskCreatePinnedToCore(CoreTask1, "CPU_1", 4096, NULL, 8, &Core1TaskHnd, 1);
  vTaskDelete(NULL);
}

void loop() {
}

void doMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 8);
  display.println(current_menu.header);
  if (current_menu.header == "Azimuth") {
    direct_control();
  } else {
    int start = current_menu.position - 3 < 0 ? 0 : current_menu.position - 3;
    int end = current_menu.position + 3 > current_menu.num_items ? current_menu.num_items : current_menu.position + 3;
    end = end < current_menu.num_items ? current_menu.num_items : end;
    for (int i = start; i < end; i++) {
      if (i == current_menu.position) {
        display.print("> ");
      } else {
        display.print("  ");
      }
      display.println(current_menu.menu_items[i].entry);
    }
  }
  display.display();
}

void from_to(float t) {
  float current_az = abs(az < 0 ? fmod(az, 360.0) + 360.0 : fmod(az, 360.0));
  if (current_az == t) return;
  float diff = fmod((current_az - t + 180), 360) - 180;
  az -= diff < -180 ? diff + 360 : diff;
  if (az > 540.0) {
    az -= 360.0;
  } else if (az < -540.0) {
    az += 360.0;
  }
}

void CoreTask0(void* parameter) {
  for (;;) {
    vTaskDelay(50);
    doMenu();

    if (encoder_changed) {
      rotary_loop();
    }
    if (encoder.isEncoderButtonClicked()) {
      rotary_onButtonClick();
    }
    if (Serial.available() >= 1) {
      Serial.println("AZ" + String(stepper.currentPosition() / steps_per_degree) + " EL" + String(el));

      serial_in = Serial.readStringUntil('\n');

      if (serial_in.charAt(2) == ' ') continue;

      split_index = serial_in.indexOf(" ");
      az_in = serial_in.substring(2, split_index);
      el_in = serial_in.substring(split_index + 3);

      from_to(az_in.toFloat());
      el = el_in.toFloat();
    }
  }
}

void CoreTask1(void* parameter) {
  for (;;) {
    //vTaskDelay(1);
    if (az != az_old) {
      stepper.moveTo(az * steps_per_degree);
      az_old = az;
    }
    stepper.run();
  }
}

void country_menu() {
  if (current_menu.header != "Country") {
    current_menu = countries_menu;
    encoder.setBoundaries(0, 243, false);
    encoder.setEncoderValue(0);
  }
}

void prefix_menu_fun() {
  if (current_menu.header != "Prefix") {
    current_menu = prefix_menu;
    encoder.setBoundaries(0, 319, false);
    encoder.setEncoderValue(0);
  }
}

void direct_control() {
  if (current_menu.header != "Azimuth") {
    current_menu = direct_menu;
    encoder.setBoundaries(0, 360, true);
    encoder.setEncoderValue(abs(az < 0 ? fmod(az, 360.0) + 360.0 : fmod(az, 360.0)));
  }
  display.println("\n");
  display.setTextSize(2);
  display.print(abs(az < 0 ? fmod(az, 360.0) + 360.0 : fmod(az, 360.0)));
  display.println((char)247);
  float disp_az = stepper.currentPosition() / steps_per_degree;
  float disp_az_norm = abs(disp_az < 0 ? fmod(disp_az, 360.0) + 360.0 : fmod(disp_az, 360.0));
  display.print(disp_az_norm);
  display.println((char)247);
  display.drawCircle(centreX, centreY, radius, WHITE);
  float dx = (radius * cos((disp_az_norm - 90) * 3.14 / 180)) + centreX;  // calculate X position for the screen coordinates - can be confusing!
  float dy = (radius * sin((disp_az_norm - 90) * 3.14 / 180)) + centreY;  // calculate Y position for the screen coordinates - can be confusing!
  draw_arrow(dx, dy, centreX, centreY, 3, 3, WHITE);
}

void stop() {
  float step_az = stepper.currentPosition() / steps_per_degree;
  float brake_az = step_az < 0 ? fmod(step_az, 360.0) + 360.0 : fmod(step_az, 360.0);
  az = brake_az;
  az_old = brake_az;
}

void set_zero() {
  az = 0.0;
  az_old = 0.0;
  encoder.reset();
  stepper.setCurrentPosition((long)0);
  direct_control();
}

void handle_country() {
  from_to(bearing(pos_lat, pos_lon, countries[current_menu.position].lat, countries[current_menu.position].lon));
}

void handle_prefix() {
  from_to(bearing(pos_lat, pos_lon, prefixes[current_menu.position].lat, prefixes[current_menu.position].lon));
}

void set_lat() {

   preferences.putFloat("pos_lat", pos_lat);
}
void set_lon() {

  preferences.putFloat("pos_lon", pos_lon);
}

float bearing(float lat, float lon, float lat2, float lon2) {
  float teta1 = radians(lat);
  float teta2 = radians(lat2);
  float delta1 = radians(lat2 - lat);
  float delta2 = radians(lon2 - lon);

  float y = sin(delta2) * cos(teta2);
  float x = cos(teta1) * sin(teta2) - sin(teta1) * cos(teta2) * cos(delta2);
  float brng = atan2(y, x);
  brng = degrees(brng);
  brng = (((int)brng + 360) % 360);
  return brng;
}

String calculateMaidenheadLocator(double latitude, double longitude) {
  const char* alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  double lat = latitude + 90.0;
  double lon = longitude + 180.0;

  // Calculate field
  int fieldLon = floor(lon / 20.0);
  int fieldLat = floor(lat / 10.0);

  // Calculate square
  int squareLon = floor((lon - fieldLon * 20.0) / 2.0);
  int squareLat = floor((lat - fieldLat * 10.0) / 1.0);

  // Calculate subsquare
  int subsquareLon = floor((lon - fieldLon * 20.0 - squareLon * 2.0) / (5.0 / 60.0));
  int subsquareLat = floor((lat - fieldLat * 10.0 - squareLat * 1.0) / (2.5 / 60.0));

  // Calculate extended subsquare
  int extSubsquareLon = floor((lon - fieldLon * 20.0 - squareLon * 2.0 - subsquareLon * (5.0 / 60.0)) / (5.0 / 60.0 / 24.0));
  int extSubsquareLat = floor((lat - fieldLat * 10.0 - squareLat * 1.0 - subsquareLat * (2.5 / 60.0)) / (2.5 / 60.0 / 24.0));

  // Construct Maidenhead Locator string
  String locator = "";
  locator += alphabet[fieldLon];
  locator += alphabet[fieldLat];
  locator += String(squareLon);
  locator += String(squareLat);
  locator += alphabet[subsquareLon];
  locator += alphabet[subsquareLat];
  locator += String(extSubsquareLon);
  locator += String(extSubsquareLat);

  return locator;
}