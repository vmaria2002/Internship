<?php

use PHPMailer\PHPMailer\PHPMailer;
use PHPMailer\PHPMailer\SMTP;
use PHPMailer\PHPMailer\Exception;
error_reporting(NULL);
require 'vendor/autoload.php';
$mail = new PHPMailer(true);
$output = '';
if (isset($_POST['submit'])) {
  $name = $_POST['name'];
  $email = $_POST['email'];
  $subject = $_POST['subject'];
  $message = $_POST['message'];
  $contactul = $_POST['contact'];

  $check = $_POST['flexCheckChecked'];

  $fileport = $_FILES['fileCV']['tmp_name'];
  $filename = $_FILES['fileCV']['name'];


  if ($check == true) {
    $subscr = "Subscribe";
  } elseif ($check == false) {
    $subscr = "Unubscribe";
  }
  if ($contactul == "1") {
    $tip = "Message";
  } elseif ($contactul == "2") {
    $tip = "Phone";
  } elseif ($contactul == "3") {
    $tip = "Email";
  }
  //
  // $file = "./img/" . $_POST['fileCV'];
  try {
    $mail->SMTPDebug = SMTP::DEBUG_SERVER;
    $mail->isSMTP();
    $mail->Host = 'in-v3.mailjet.com';
    $mail->SMTPAuth = true;
    $mail->Username = '748d4851fe0c554f2ead0478f5c41e70';
    $mail->Password = 'cf5232c2ebb48ea4686f7f4db6f2423a';
    $mail->SMTPSecure = 'tls';
    $mail->Port = 587;
    $mail->setFrom('maria.vasilache02@gmail.com');
    $mail->addAddress('vasilachemaria20@yahoo.com');
    $mail->addAddress( $email);
    $mail->isHTML(true);
    $mail->Subject = 'Form Submission';
    $mail->Body = "<h3>Name : $name <br>Email : $email <br>Message : $message <br> Subscribe: $subscr <br> Contact with: $tip </h3>";
  
    $mail->addAttachment($fileport,$filename);



    $result = $mail->send();
    $output = '<div class="alert alert-success">
                  <h5>Thankyou! for contacting us, Well get back to you soon!</h5>
                </div>';
  } catch (Exception $e) {
    $output = '<div class="alert alert-danger">
                  <h5>' . $e->getMessage() . '</h5>
                </div>';
  }
}
?>
<!DOCTYPE html>
<html lang="en">

<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Contact Us Using PHPMailer & Gmail SMTP</title>
  <link rel='stylesheet' href='https://cdnjs.cloudflare.com/ajax/libs/twitter-bootstrap/4.5.0/css/bootstrap.min.css' />
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Formular</title>
  <link rel="stylesheet" href="css/style.css">
  <link rel="stylesheet" href="https://unpkg.com/swiper/swiper-bundle.min.css" />
  <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.0.2/dist/css/bootstrap.min.css" rel="stylesheet" integrity="sha384-EVSTQN3/azprG1Anm3QDgpJLIm9Nao0Yz1ztcQTwFspd3yD65VohhpuuCOmLASjC" crossorigin="anonymous">
  <link href="https://fonts.googleapis.com/icon?family=Material+Icons" rel="stylesheet">
  <link href="https://unpkg.com/aos@2.3.1/dist/aos.css" rel="stylesheet">
</head>

<body>
  <header>
    <?php include "header.php"; ?>
  </header>
</body>

<div class="bg-dark">
  <div class="container">
    <div class="row justify-content-center">
      <div class="col-lg-6 mt-3">
        <div class="card border-danger shadow">
          <div class="card-header bg-danger text-light">
            <h3 class="card-title" data-aos="fade-up">Contact Us</h3>
          </div>
          <div class="card-body px-4">
            <form enctype="multipart/form-data" action="#" method="POST">
              <div class="form-group" data-aos="fade-up">
                <?= $output; ?>
              </div>
              <div class="form-group" data-aos="fade-up">
                <label for="name">Name</label>
                <input type="text" name="name" id="name" class="form-control" placeholder="Enter Name" required>
              </div>
              <div class="form-group" data-aos="fade-up">
                <label for="email">E-Mail</label>
                <input type="email" name="email" id="email" class="form-control" placeholder="Enter E-Mail" required>
              </div>
              <div class="form-group" data-aos="fade-up">
                <label for="subject">Phone Number</label>
                <input type="text" name="subject" id="subject" class="form-control" placeholder="Enter Phone Number" required>
              </div>
              <div class="form-group" data-aos="fade-up">
                <label for="message" data-aos="fade-up">Message</label>
                <textarea name="message" id="message" rows="5" class="form-control" placeholder="Write Your Message" required></textarea>
              </div>
              <div class="form-check" data-aos="fade-up">
                <input type="checkbox" name="flexCheckChecked" id="flexCheckChecked" class="form-check-input"  checked>
                <label class="form-check-label" for="flexCheckChecked">
                  Do you want to subscribe to our newsletter?
                </label>
              </div>
              <br></br>
              <label data-aos="fade-up">How do you want to be contacted?</label>
              <select class="form-select" aria-label="Default select example" id="contact" name="contact">
                <option value="1">Mesage</option>
                <option value="2">Phone</option>
                <option value="3">Email</option>
              </select>
              <br></br>

              <div class="mb-3" data-aos="fade-up">
                <label for="file">Please attach your CV / resume in PDF format</label>
                <br></br>
                <input type="file" accept='.pdf' name="fileCV" id="fileCV" class="firm-control">
              </div> <br>
              <div class="form-group" data-aos="fade-up">
                <input type="submit" name="submit" value="Send" class="btn btn-danger btn-block" id="sendBtn">
              </div>
            </form>
          </div>
        </div>
      </div>
    </div>

  </div>
  </body>
  <footer>
    <?php include "footer.php"; ?>
  </footer>

  <script src="js/custom.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.0.2/dist/js/bootstrap.bundle.min.js" integrity="sha384-MrcW6ZMFYlzcLA8Nl+NtUVF0sA7MsXsP1UyJoMp4YLEuNSfAP+JcXn/tWtIaxVXM" crossorigin="anonymous"></script>
  <script src="https://unpkg.com/aos@2.3.1/dist/aos.js"></script>
  <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/4.7.0/css/font-awesome.min.css">

  <script>
    AOS.init();
  </script>
  <link rel="stylesheet" href="css/font-awesome.min.css">
  </body>

</html>

</html>